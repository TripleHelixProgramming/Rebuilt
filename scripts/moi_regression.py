#!/usr/bin/env python3
"""
MOI regression for simulated mechanisms using all usable VAALE WPILog files.

Physics:  G * n * kt * I  =  J * alpha  +  B * omega
Solve 2-parameter OLS for [J, B].

Reads log files directly using robotpy-wpiutil DataLogReader.
Computes alpha/omega per-log to avoid cross-log boundary artifacts,
then pools all filtered samples across all logs for a single OLS fit.

Conventions:
  - Flywheel/Kicker/Spindexer: logged as linear velocity (m/s); wheel radius r converts to angular
  - Turret: logged as angular velocity (rad/s) at output shaft directly
  - Turret current requires sign(AppliedVolts) because SparkMax reports unsigned current
"""
import os
import sys
import numpy as np
from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Log directory

LOG_DIR = r"C:\Users\natel\Documents\robotLogs\2026\VAALE"

# p6  = idle/practice - robot never ran mechanisms
# q38 = never enabled (11 MB, essentially empty)
SKIP_LOGS = {
    "akit_26-03-07_15-16-23_vaale_p6.wpilog",
    "akit_26-03-07_21-59-02_vaale_q38.wpilog",
}

# ---------------------------------------------------------------------------
# Mechanism configs
#
#   tau_scale = G * n * kt  (Nm per amp at output shaft)
#   For linear-velocity logs: omega = vel / wheel_radius,  alpha = d(omega)/dt

MECHANISMS = {
    "flywheel": dict(
        vel_entry="/Flywheel/VelocityMetersPerSec",
        curr_entry="/Flywheel/CurrentAmps",
        volts_entry=None,
        G=1.0, n_motors=2, kt=0.01940,  # 2x Kraken X60
        wheel_radius=0.0381,             # 1.5 in  (LauncherConstants)
        smooth_hw=5, alpha_thresh=30.0, label="Flywheel",
    ),
    "kicker": dict(
        vel_entry="/Kicker/VelocityMetersPerSec",
        curr_entry="/Kicker/CurrentAmps",
        volts_entry=None,
        G=1.0, n_motors=1, kt=0.01706,  # NEO Vortex
        wheel_radius=0.0381,             # 1.5 in  (FeederConstants.KickerConstants)
        smooth_hw=3, alpha_thresh=5.0, label="Kicker",
    ),
    "spindexer": dict(
        vel_entry="/Spindexer/VelocityMetersPerSec",
        curr_entry="/Spindexer/CurrentAmps",
        volts_entry=None,
        G=1.0, n_motors=1, kt=0.01706,  # NEO Vortex
        wheel_radius=0.0762,             # 3.0 in  (FeederConstants.SpindexerConstants)
        smooth_hw=3, alpha_thresh=5.0, label="Spindexer",
    ),
    "turret": dict(
        vel_entry="/Turret/VelocityRadPerSec",
        curr_entry="/Turret/CurrentAmps",
        volts_entry="/Turret/AppliedVolts",   # sign needed: SparkMax unsigned current
        G=54.0, n_motors=1, kt=0.0108,        # NEO 550
        wheel_radius=None,                     # already rad/s at output shaft
        smooth_hw=3, alpha_thresh=1.0, label="Turret",
    ),
}

# ---------------------------------------------------------------------------
# Signal processing helpers


def smooth(vals, hw):
    """Symmetric moving-average, half-width hw samples."""
    if hw <= 0:
        return vals.copy()
    kernel = np.ones(2 * hw + 1) / (2 * hw + 1)
    return np.convolve(vals, kernel, mode="same")


def gradient_cd(vals, ts):
    """Central-difference derivative; forward/backward at endpoints."""
    n = len(vals)
    alpha = np.empty(n)
    if n < 2:
        alpha[:] = 0.0
        return alpha
    dt_fwd = ts[2:] - ts[:-2]
    alpha[1:-1] = np.where(dt_fwd > 1e-9, (vals[2:] - vals[:-2]) / dt_fwd, 0.0)
    dt0 = ts[1] - ts[0]
    alpha[0] = (vals[1] - vals[0]) / dt0 if dt0 > 1e-9 else 0.0
    dt_e = ts[-1] - ts[-2]
    alpha[-1] = (vals[-1] - vals[-2]) / dt_e if dt_e > 1e-9 else 0.0
    return alpha


def nearest_interp(ts_query, ts_ref, vals_ref):
    """Nearest-neighbour interpolation."""
    n = min(len(ts_ref), len(vals_ref))
    ts_ref   = np.asarray(ts_ref[:n],   float)
    vals_ref = np.asarray(vals_ref[:n], float)
    idx = np.searchsorted(ts_ref, ts_query).clip(0, n - 1)
    left = idx > 0
    closer_left = left & (np.abs(ts_ref[idx - 1] - ts_query) < np.abs(ts_ref[idx] - ts_query))
    idx[closer_left] -= 1
    return vals_ref[idx]


# ---------------------------------------------------------------------------
# OLS solver


def ols_pooled(alpha_all, omega_all, torque_all, label):
    """2-parameter OLS: torque = J*alpha + B*omega."""
    ok = np.isfinite(alpha_all) & np.isfinite(omega_all) & np.isfinite(torque_all)
    a, w, t = alpha_all[ok], omega_all[ok], torque_all[ok]
    if len(t) < 10:
        print(f"  [{label}] INSUFFICIENT SAMPLES: {len(t)}")
        return None, None, None
    X = np.column_stack([a, w])
    sol, *_ = np.linalg.lstsq(X, t, rcond=None)
    J, B = sol
    y_hat  = X @ sol
    ss_res = np.sum((t - y_hat) ** 2)
    ss_tot = np.sum((t - t.mean()) ** 2)
    r2 = 1 - ss_res / ss_tot if ss_tot > 1e-15 else float("nan")
    print(f"  [{label}] n={len(t):5d}  J={J:.6f} kg*m^2  B={B:.8f} Nm*s/rad  R^2={r2:.4f}")
    if J < 0:
        print(f"    WARNING: negative J -- check signed-current convention")
    if not np.isnan(r2) and r2 < 0.2:
        print(f"    WARNING: low R^2 -- consider adjusting alpha_thresh")
    return J, B, r2


# ---------------------------------------------------------------------------
# Log reader


def read_log_series(log_path):
    """
    Parse a WPILOG and return {entry_name: (timestamps_s, values)} for double entries.
    Timestamps are converted from microseconds to seconds.
    """
    reader = DataLogReader(log_path)
    id_to_name, id_to_type, raw = {}, {}, {}
    for rec in reader:
        if rec.isStart():
            s = rec.getStartData()
            id_to_name[s.entry] = s.name
            id_to_type[s.entry] = s.type
        elif not rec.isControl():
            eid = rec.getEntry()
            if id_to_type.get(eid) != "double":
                continue
            name = id_to_name.get(eid)
            if name is None:
                continue
            ts  = rec.getTimestamp() / 1e6  # us -> s
            val = rec.getDouble()
            if name not in raw:
                raw[name] = ([], [])
            raw[name][0].append(ts)
            raw[name][1].append(val)
    result = {}
    for name, (ts_list, val_list) in raw.items():
        ts  = np.asarray(ts_list, float)
        val = np.asarray(val_list, float)
        order = np.argsort(ts)
        result[name] = (ts[order], val[order])
    return result


# ---------------------------------------------------------------------------
# Per-log sample extractor


def extract_samples(series, cfg):
    """
    Return (alpha, omega, torque) for samples with |alpha| > alpha_thresh.
    Derivatives are computed within this log only (no cross-log boundary artifacts).
    """
    vel_key   = cfg["vel_entry"]
    curr_key  = cfg["curr_entry"]
    volts_key = cfg["volts_entry"]
    if vel_key not in series or curr_key not in series:
        return np.array([]), np.array([]), np.array([])
    if volts_key and volts_key not in series:
        return np.array([]), np.array([]), np.array([])

    vel_ts, vel_raw   = series[vel_key]
    curr_ts, curr_raw = series[curr_key]

    r = cfg["wheel_radius"]
    omega_raw = vel_raw / r if r is not None else vel_raw

    hw = cfg["smooth_hw"]
    omega_sm  = smooth(omega_raw, hw)
    alpha_raw = gradient_cd(omega_sm, vel_ts)
    alpha_sm  = smooth(alpha_raw, hw)

    curr_interp = nearest_interp(vel_ts, curr_ts, curr_raw)
    tau_scale   = cfg["G"] * cfg["n_motors"] * cfg["kt"]

    if volts_key:
        volts_ts, volts_raw = series[volts_key]
        volts_interp = nearest_interp(vel_ts, volts_ts, volts_raw)
        torque = tau_scale * np.sign(volts_interp) * curr_interp
    else:
        torque = tau_scale * curr_interp

    mask = np.abs(alpha_sm) > cfg["alpha_thresh"]
    return alpha_sm[mask], omega_sm[mask], torque[mask]


# ---------------------------------------------------------------------------
# Main


def main():
    if not os.path.isdir(LOG_DIR):
        print(f"ERROR: log directory not found: {LOG_DIR}", file=sys.stderr)
        sys.exit(1)

    log_files = sorted(
        f for f in os.listdir(LOG_DIR)
        if f.endswith(".wpilog") and f not in SKIP_LOGS
    )
    print(f"Found {len(log_files)} usable logs in {LOG_DIR}")
    print()

    pools      = {k: ([], [], []) for k in MECHANISMS}
    log_counts = {k: [] for k in MECHANISMS}

    for fname in log_files:
        path = os.path.join(LOG_DIR, fname)
        print(f"  Reading {fname} ...")
        try:
            series = read_log_series(path)
        except Exception as e:
            print(f"    SKIP (read error): {e}")
            continue
        for mkey, cfg in MECHANISMS.items():
            a, w, t = extract_samples(series, cfg)
            n = len(a)
            log_counts[mkey].append((fname, n))
            if n > 0:
                pools[mkey][0].extend(a)
                pools[mkey][1].extend(w)
                pools[mkey][2].extend(t)

    print()
    print("=" * 62)
    print("  POOLED OLS RESULTS")
    print("=" * 62)

    results = {}
    for mkey, cfg in MECHANISMS.items():
        label     = cfg["label"]
        tau_scale = cfg["G"] * cfg["n_motors"] * cfg["kt"]
        r_val     = cfg["wheel_radius"]
        G         = cfg["G"]
        nm        = cfg["n_motors"]
        kt        = cfg["kt"]
        at        = cfg["alpha_thresh"]
        print()
        print(f"{label.upper()}  (G={G}, n={nm}, kt={kt}, tau_scale={tau_scale:.4f} Nm/A)")
        if r_val:
            print(f"  wheel_radius={r_val*1000:.1f} mm,  alpha_thresh={at} rad/s^2")
        else:
            print(f"  angular log,  alpha_thresh={at} rad/s^2")

        total = sum(n for _, n in log_counts[mkey])
        if total == 0:
            print("  No samples found -- check entry names.")
            results[mkey] = (None, None, None)
            continue

        for fname, n in log_counts[mkey]:
            if n > 0:
                short = fname.replace("akit_26-03-0", "").replace(".wpilog", "")
                print(f"    {short:<44s}  {n:5d} samples")
        total_label = "TOTAL"
        print(f"    {total_label:<44s}  {total:5d} samples")

        a_all = np.asarray(pools[mkey][0])
        w_all = np.asarray(pools[mkey][1])
        t_all = np.asarray(pools[mkey][2])
        J, B, r2 = ols_pooled(a_all, w_all, t_all, label)
        results[mkey] = (J, B, r2)

    print()
    print("=" * 62)
    print("  JAVA CONSTANTS SUMMARY")
    print("=" * 62)
    java_names = {
        "flywheel":  "FLYWHEEL_MOI_KG_M2",
        "kicker":    "KICKER_MOI_KG_M2",
        "spindexer": "SPINDEXER_MOI_KG_M2",
        "turret":    "TURRET_MOI_KG_M2",
    }
    for mkey, jname in java_names.items():
        J, B, r2 = results.get(mkey, (None, None, None))
        if J is not None:
            print(f"  private static final double {jname} = {J:.5g};  // R^2={r2:.3f}")
        else:
            print(f"  private static final double {jname} = ???;  // regression failed")


if __name__ == "__main__":
    main()
