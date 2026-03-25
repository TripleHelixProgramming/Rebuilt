#!/usr/bin/env python3
"""
FRC Robot Energy Consumption Analysis — VACHE (all competition matches)

Creates a stacked area plot of *average* cumulative energy consumed (kJ) by
each robot mechanism over match elapsed time, across all usable VACHE matches.

  Energy = integral(V * I  dt)  per mechanism group
  Aligned to t=0 at autonomous start for each match.
  Averaged across all full matches (Q13 excluded — no teleop).

Requires: robotpy-wpiutil, numpy, matplotlib
"""

from pathlib import Path
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker
from wpiutil.log import DataLogReader

# ---------------------------------------------------------------------------
# Match index — (log_path, auto_start_s, match_end_s)
# Phase times from get_match_phases; Q13 excluded (incomplete — auto only).

LOG_ROOT = r"C:\Users\natel\wpilib\logs\VACHE"

MATCHES = [
    # (session folder,              filename,                            auto_start,  match_end)
    ("session_20", "akit_26-03-21_15-28-32_vache_q4.wpilog",   293.06772,  457.987571),
    ("session_23", "akit_26-03-21_16-29-56_vache_q10.wpilog",  110.991153, 278.380621),
    ("session_29", "akit_26-03-21_18-58-00_vache_q20.wpilog",  167.10684,  332.024186),
    ("session_31", "akit_26-03-21_19-22-32_vache_q23.wpilog",  122.673832, 287.730266),
    ("session_33", "akit_26-03-21_20-05-00_vache_q27.wpilog",  459.106031, 623.176552),
    ("session_35", "akit_26-03-21_20-51-53_vache_q32.wpilog",  306.60878,  471.251957),
    ("session_38", "akit_26-03-21_21-29-57_vache_q36.wpilog",  134.091819, 298.317325),
    ("session_43", "akit_26-03-22_13-41-47_vache_q45.wpilog",  78.614195,  243.695301),
    ("session_45", "akit_26-03-22_14-06-50_vache_q48.wpilog",  301.254757, 465.837572),
    ("session_46", "akit_26-03-22_14-57-55_vache_q54.wpilog",  150.011873, 314.890358),
    ("session_49", "akit_26-03-22_15-33-11_vache_q58.wpilog",  115.237395, 279.559398),
    ("session_55", "akit_26-03-22_18-15-22_vache_e4.wpilog",   143.041149, 307.546705),
    ("session_57", "akit_26-03-22_18-52-56_vache_e8.wpilog",   142.076896, 306.130295),
    ("session_60", "akit_26-03-22_19-16-43_vache_e10.wpilog",  97.628853,  262.675419),
]

# Average auto-end elapsed time (seconds into match) across all logs
AVG_AUTO_ELAPSED_S = 24.4

# Common output grid: 0 → 165 s elapsed at 50 Hz
GRID_DT_S      = 0.02
GRID_DURATION  = 165.0
COMMON_GRID    = np.arange(0.0, GRID_DURATION + GRID_DT_S, GRID_DT_S)

VOLTAGE_ENTRY      = "/RealOutputs/PDH/Voltage"
PDH_CHANNELS_ENTRY = "/RealOutputs/PDH/ChannelCurrentsAmps"
NOMINAL_VOLTAGE    = 12.0  # V fallback

# PDH channel allocation (0-indexed, 24 channels)
#   ch 0-1   : back-left drive/turn        → Drive
#   ch 2     : turret                      → Turret
#   ch 3     : hood                        → Hood
#   ch 4-5   : flywheel leader/follower    → Flywheel
#   ch 6     : spindexer                   → Spindexer
#   ch 7     : kicker                      → Kicker
#   ch 8-9   : back-right drive/turn       → Drive
#   ch 10-11 : front-right drive/turn      → Drive
#   ch 12-13 : intake roller left/right    → Lower/Upper Roller
#   ch 14-15 : OrangePis / LEDs            → Electronics (use PDH)
#   ch 16-17 : unused
#   ch 18-19 : front-left drive/turn       → Drive
#   ch 20    : RIO                         → Electronics (use PDH)
#   ch 21    : Radio                       → Electronics (use PDH)
#   ch 22    : CANivore/coprocessors/LEDs  → Electronics (use PDH)
#   ch 23    : PCH (compressor + overhead) → Compressor (AK) + PCH overhead (PDH delta)
PDH_ELECTRONICS_CHANNELS = [14, 15, 20, 21, 22]  # OrangePis/LEDs + RIO/radio/CANivore/coprocessors

# Mechanism groups sourced from AdvantageKit motor-controller entries
MECHANISM_GROUPS = {
    "Drive": [
        "/Drive/Module0/DriveCurrentAmps",
        "/Drive/Module0/TurnCurrentAmps",
        "/Drive/Module1/DriveCurrentAmps",
        "/Drive/Module1/TurnCurrentAmps",
        "/Drive/Module2/DriveCurrentAmps",
        "/Drive/Module2/TurnCurrentAmps",
        "/Drive/Module3/DriveCurrentAmps",
        "/Drive/Module3/TurnCurrentAmps",
    ],
    "Flywheel":     ["/Flywheel/CurrentAmps", "/Flywheel/FollowerCurrentAmps"],
    "Turret":       ["/Turret/CurrentAmps"],
    "Spindexer":    ["/Spindexer/CurrentAmps"],
    "Kicker":       ["/Kicker/CurrentAmps"],
    "Lower Roller": ["/LowerRoller/CurrentAmps"],
    "Upper Roller": ["/UpperRoller/CurrentAmps"],
    "Hood":         ["/Hood/CurrentAmps"],
    "Compressor":   ["/RealOutputs/Compressor/CurrentAmps"],
    # Overhead groups sourced from PDH channel array (no AK motor entries)
    # Listed here so accum is initialized for them; currents come from PDH in process_match()
    "Electronics": [],  # OrangePis/LEDs + RIO/radio/CANivore/coprocessors — sourced from PDH channels
}

# Mechanisms using Krakens (TalonFX/Phoenix 6) whose supply current can go
# negative during regenerative braking.  Energy is integrated without clamping
# for these groups so regen recovery reduces the net energy total.
KRAKEN_GROUPS = {"Drive", "Flywheel", "Lower Roller", "Upper Roller"}

# ---------------------------------------------------------------------------
# Log reader


def read_log(log_path: str) -> tuple[
    dict[str, tuple[np.ndarray, np.ndarray]],       # scalar doubles
    dict[str, tuple[np.ndarray, list]],              # double arrays  {name: (ts, [vec, ...])}
]:
    """
    Parse a WPILOG and return:
      scalars : {entry_name: (timestamps_s, values_1d)}   for double entries
      arrays  : {entry_name: (timestamps_s, list_of_vecs)} for double[] entries
    """
    reader = DataLogReader(log_path)
    id_to_name: dict[int, str] = {}
    id_to_type: dict[int, str] = {}
    raw_s: dict[str, tuple[list, list]] = {}
    raw_a: dict[str, tuple[list, list]] = {}

    for rec in reader:
        if rec.isStart():
            s = rec.getStartData()
            id_to_name[s.entry] = s.name
            id_to_type[s.entry] = s.type
        elif not rec.isControl():
            eid  = rec.getEntry()
            typ  = id_to_type.get(eid)
            name = id_to_name.get(eid)
            if name is None:
                continue
            ts = rec.getTimestamp() / 1e6
            if typ == "double":
                if name not in raw_s:
                    raw_s[name] = ([], [])
                raw_s[name][0].append(ts)
                raw_s[name][1].append(rec.getDouble())
            elif typ == "double[]":
                if name not in raw_a:
                    raw_a[name] = ([], [])
                raw_a[name][0].append(ts)
                raw_a[name][1].append(rec.getDoubleArray())

    scalars: dict[str, tuple[np.ndarray, np.ndarray]] = {}
    for name, (ts_list, val_list) in raw_s.items():
        ts  = np.asarray(ts_list, float)
        val = np.asarray(val_list, float)
        order = np.argsort(ts)
        scalars[name] = (ts[order], val[order])

    arrays: dict[str, tuple[np.ndarray, list]] = {}
    for name, (ts_list, vec_list) in raw_a.items():
        ts    = np.asarray(ts_list, float)
        order = np.argsort(ts)
        arrays[name] = (ts[order], [vec_list[i] for i in order])

    return scalars, arrays


def extract_pdh_channel(
    arrays: dict[str, tuple[np.ndarray, list]],
    channels: list[int],
    t_abs: np.ndarray,
) -> np.ndarray:
    """Sum the given PDH channel indices onto t_abs. Returns zeros if unavailable."""
    if PDH_CHANNELS_ENTRY not in arrays:
        return np.zeros_like(t_abs)
    ts, vecs = arrays[PDH_CHANNELS_ENTRY]
    # Reconstruct as 2D array: shape (n_samples, n_channels)
    n_ch = len(vecs[0]) if vecs else 24
    mat = np.array([v[:n_ch] for v in vecs], dtype=float)  # (n_samples, n_ch)
    combined = mat[:, channels].sum(axis=1)                 # sum selected channels
    return np.interp(t_abs, ts, combined, left=combined[0], right=combined[-1])


# ---------------------------------------------------------------------------
# Per-match energy computation


def interp_entry(
    series: dict[str, tuple[np.ndarray, np.ndarray]],
    entry: str,
    t_abs: np.ndarray,
) -> np.ndarray:
    """Interpolate entry onto absolute time grid; return zeros if missing."""
    if entry not in series:
        return np.zeros_like(t_abs)
    ts, vs = series[entry]
    mask = (ts >= t_abs[0] - 1.0) & (ts <= t_abs[-1] + 1.0)
    ts, vs = ts[mask], vs[mask]
    if len(ts) == 0:
        return np.zeros_like(t_abs)
    return np.interp(t_abs, ts, vs, left=vs[0], right=vs[-1])


def cumulative_energy_J(
    t_abs: np.ndarray, current: np.ndarray, voltage: np.ndarray,
    clamp: bool = False,
) -> np.ndarray:
    """Trapezoid-integrate P=V*I. Returns J array.
    clamp=True: floor supply current at 0 (use for non-regen sources like PDH channels).
    clamp=False (default): allow negative current so regen braking reduces net energy.
    """
    i = np.maximum(current, 0.0) if clamp else current
    power = i * voltage
    dE    = 0.5 * (power[:-1] + power[1:]) * np.diff(t_abs)
    return np.concatenate([[0.0], np.cumsum(dE)])


def process_match(
    log_path: str, auto_start: float, match_end: float
) -> dict[str, np.ndarray] | None:
    """
    Return {group_name: energy_kJ_on_COMMON_GRID} for one match, or None on failure.
    Energy is computed on an absolute time grid [auto_start, match_end],
    then the cumulative curve is resampled onto COMMON_GRID (elapsed time).
    After the match ends, the final energy value is held constant.
    """
    scalars, arrays = read_log(log_path)

    t_abs = np.arange(auto_start, match_end + GRID_DT_S, GRID_DT_S)

    # Battery voltage
    voltage = interp_entry(scalars, VOLTAGE_ENTRY, t_abs)
    if voltage.max() < 1.0:
        voltage = np.full_like(t_abs, NOMINAL_VOLTAGE)

    match_duration = match_end - auto_start   # seconds

    def to_common(current: np.ndarray, clamp: bool = False) -> np.ndarray:
        energy_kJ = cumulative_energy_J(t_abs, current, voltage, clamp=clamp) / 1000.0
        t_elapsed  = np.linspace(0.0, match_duration, len(t_abs))
        return np.interp(COMMON_GRID, t_elapsed, energy_kJ,
                         left=0.0, right=energy_kJ[-1])

    result: dict[str, np.ndarray] = {}

    PDH_SOURCED = {"Electronics"}

    # AK motor-controller groups (skip PDH-sourced placeholders)
    for group, entries in MECHANISM_GROUPS.items():
        if group in PDH_SOURCED:
            continue
        total_current = np.zeros_like(t_abs)
        found = False
        for entry in entries:
            if entry in scalars:
                current = interp_entry(scalars, entry, t_abs)
                # AKit delta-logs current only on value change. Motors idle at
                # exactly 0 A produce no log entries, so the first sample in
                # our window may be non-zero (e.g. mid-shoot for spindexer/
                # kicker). interp_entry extrapolates that value back to t=0,
                # falsely accumulating energy. Zero out everything before the
                # first actual logged timestamp within this match window.
                ts_all = scalars[entry][0]
                in_window = ts_all[(ts_all >= auto_start - 1.0) & (ts_all <= match_end + 1.0)]
                if len(in_window) > 0:
                    current[t_abs < in_window[0]] = 0.0
                total_current += current
                found = True
        clamp = group not in KRAKEN_GROUPS
        result[group] = to_common(total_current, clamp=clamp) if found else np.zeros_like(COMMON_GRID)

    # PDH-sourced overhead group (climber + RIO/radio/coprocessors/LEDs)
    electronics_i = extract_pdh_channel(arrays, PDH_ELECTRONICS_CHANNELS, t_abs)
    result["Electronics"] = to_common(electronics_i)

    return result


# ---------------------------------------------------------------------------
# Plotting helper


def plot_energy(
    energies: dict[str, np.ndarray],
    title: str,
    auto_elapsed_s: float,
    out_path: Path,
    n_matches: int = 1,
):
    """Render and save a stacked area energy plot (kJ, largest on top)."""
    # Smallest at bottom → largest on top
    sorted_names = sorted(energies, key=lambda g: energies[g][-1], reverse=False)

    total_kJ = sum(energies[g][-1] for g in sorted_names)
    print(f"\nEnergy breakdown (end of match):")
    for name in sorted_names[::-1]:   # print largest first
        e = energies[name][-1]
        print(f"  {name:20s}  {e:7.1f} kJ  ({100*e/total_kJ:.1f}%)")
    print(f"  {'TOTAL':20s}  {total_kJ:7.1f} kJ")

    fig, ax = plt.subplots(figsize=(13, 6))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#16213e")

    cmap   = plt.get_cmap("tab10")
    colors = [cmap(i % 10) for i in range(len(sorted_names))]

    stack = np.zeros_like(COMMON_GRID)
    for i, name in enumerate(sorted_names):
        top   = stack + energies[name]
        lbl   = f"{name}  ({energies[name][-1]:.1f} kJ)"
        ax.fill_between(COMMON_GRID, stack, top, color=colors[i], alpha=0.82, label=lbl)
        ax.plot(COMMON_GRID, top, color=colors[i], linewidth=0.6)
        stack = top

    y_top = stack[-1]
    ax.axvline(auto_elapsed_s, color="white", linestyle="--", linewidth=1.0, alpha=0.7)
    ax.text(auto_elapsed_s - 0.8, y_top * 0.97, "Auto",
            color="white", fontsize=9, va="top", ha="right", alpha=0.85)
    ax.text(auto_elapsed_s + 0.8, y_top * 0.97, "Teleop",
            color="white", fontsize=9, va="top", ha="left", alpha=0.85)

    ax.set_xlabel("Match elapsed time (s)", color="white", fontsize=12)
    ax.set_ylabel("Cumulative energy (kJ)", color="white", fontsize=12)
    ax.set_title(title, color="white", fontsize=13)
    ax.tick_params(colors="white")
    for spine in ax.spines.values():
        spine.set_edgecolor("#444466")
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x:.0f}"))
    ax.set_xlim(0, GRID_DURATION)
    ax.set_ylim(bottom=0)

    handles, labels = ax.get_legend_handles_labels()
    ax.legend(
        handles[::-1], labels[::-1],
        loc="upper left", fontsize=9,
        facecolor="#1a1a2e", edgecolor="#444466", labelcolor="white",
    )

    plt.tight_layout()
    plt.savefig(out_path, dpi=150, facecolor=fig.get_facecolor())
    print(f"\nSaved -> {out_path}")
    plt.show()


# ---------------------------------------------------------------------------
# Main


def main():
    import sys

    if len(sys.argv) >= 4:
        # Single-match mode: energy_analysis.py <log_path> <auto_start_s> <match_end_s>
        log_path   = sys.argv[1]
        auto_start = float(sys.argv[2])
        match_end  = float(sys.argv[3])
        label = Path(log_path).stem
        print(f"Single match: {label}")
        result = process_match(log_path, auto_start, match_end)
        auto_elapsed = 24.4   # approximate; refine per-match if needed
        out = Path(__file__).parent.parent / "doc" / "assets" / "energy_q54.png"
        title = f"VACHE {label.split('_vache_')[-1].upper()} — Robot Energy Consumption by Mechanism"
        plot_energy(result, title, auto_elapsed, out)

    else:
        # Multi-match average mode
        print(f"Processing {len(MATCHES)} VACHE competition matches...\n")
        accum: dict[str, list[np.ndarray]] = {g: [] for g in MECHANISM_GROUPS}

        for session, filename, auto_start, match_end in MATCHES:
            log_path = str(Path(LOG_ROOT) / session / filename)
            lbl = filename.split("_vache_")[-1].replace(".wpilog", "").upper()
            print(f"  {lbl:6s}  {filename}")
            result = process_match(log_path, auto_start, match_end)
            if result is None:
                print(f"         -> SKIPPED")
                continue
            for group in MECHANISM_GROUPS:
                accum[group].append(result[group])

        n = len(next(iter(accum.values())))
        print(f"\nAveraging across {n} matches...")

        avg: dict[str, np.ndarray] = {
            group: np.mean(curves, axis=0)
            for group, curves in accum.items()
            if curves
        }

        out   = Path(__file__).parent.parent / "doc" / "assets" / "energy_average_vache.png"
        title = f"VACHE — Average Robot Energy Consumption by Mechanism  (n={n} matches)"
        plot_energy(avg, title, AVG_AUTO_ELAPSED_S, out, n_matches=n)


if __name__ == "__main__":
    main()
