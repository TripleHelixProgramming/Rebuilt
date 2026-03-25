#!/usr/bin/env python3
"""
Spindexer + Kicker cumulative energy — VACHE Q54 autonomous only.
Reuses read_log / cumulative_energy_J from energy_analysis.py.
"""

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).parent))
from energy_analysis import read_log, interp_entry, cumulative_energy_J

import numpy as np
import matplotlib.pyplot as plt
import matplotlib.ticker as ticker

LOG_PATH   = r"C:\Users\natel\wpilib\logs\VACHE\session_46\akit_26-03-22_14-57-55_vache_q54.wpilog"
AUTO_START = 150.011873
AUTO_END   = 174.650002   # autonomous end == teleop start

GRID_DT   = 0.02
VOLTAGE_ENTRY = "/RealOutputs/PDH/Voltage"
NOMINAL_V     = 12.0

MECHANISMS = {
    "Spindexer": "/Spindexer/CurrentAmps",
    "Kicker":    "/Kicker/CurrentAmps",
}

COLORS = {"Spindexer": "#4e9af1", "Kicker": "#f1884e"}


def main():
    scalars, _ = read_log(LOG_PATH)

    t_abs = np.arange(AUTO_START, AUTO_END + GRID_DT, GRID_DT)
    t_elapsed = t_abs - AUTO_START                  # 0 → ~24.6 s

    voltage = interp_entry(scalars, VOLTAGE_ENTRY, t_abs)
    if voltage.max() < 1.0:
        voltage = np.full_like(t_abs, NOMINAL_V)

    energies = {}
    for name, entry in MECHANISMS.items():
        current = interp_entry(scalars, entry, t_abs)
        # AKit delta-logs current only on value change. Motors idle at exactly 0 A
        # produce no log entries, so the first sample in our window is non-zero
        # (mid-shoot). interp_entry extrapolates that value back to t=0, falsely
        # accumulating energy during the pre-shoot idle window.
        # Fix: find the first timestamp *within this analysis window* and zero out
        # everything before it.
        if entry in scalars:
            ts_all = scalars[entry][0]
            in_window = ts_all[(ts_all >= AUTO_START - 1.0) & (ts_all <= AUTO_END + 1.0)]
            if len(in_window) > 0:
                current[t_abs < in_window[0]] = 0.0
        energies[name] = cumulative_energy_J(t_abs, current, voltage, clamp=True) / 1000.0   # → kJ

    # ── print summary ──────────────────────────────────────────────────────────
    print("\nQ54 autonomous energy (spindexer + kicker):")
    total = sum(e[-1] for e in energies.values())
    for name, e in energies.items():
        print(f"  {name:12s}  {e[-1]:.2f} kJ  ({100*e[-1]/total:.1f}%)")
    print(f"  {'TOTAL':12s}  {total:.2f} kJ")

    # ── plot ───────────────────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(10, 5))
    fig.patch.set_facecolor("#1a1a2e")
    ax.set_facecolor("#16213e")

    stack = np.zeros_like(t_elapsed)
    for name in ["Spindexer", "Kicker"]:
        top = stack + energies[name]
        lbl = f"{name}  ({energies[name][-1]:.2f} kJ)"
        ax.fill_between(t_elapsed, stack, top, color=COLORS[name], alpha=0.82, label=lbl)
        ax.plot(t_elapsed, top, color=COLORS[name], linewidth=0.7)
        stack = top

    # annotate shoot window (feeder command starts at 160.321 s)
    shoot_start = 160.321481 - AUTO_START   # ≈ 10.3 s elapsed
    shoot_end   = 165.5      - AUTO_START   # ≈ 15.5 s elapsed (current drops to idle)
    ax.axvspan(shoot_start, shoot_end, color="white", alpha=0.07, label="Shoot window")
    ax.axvline(shoot_start, color="white", linestyle="--", linewidth=0.9, alpha=0.6)
    ax.axvline(shoot_end,   color="white", linestyle="--", linewidth=0.9, alpha=0.6)
    y_top = stack[-1]
    ax.text(shoot_start + 0.2, y_top * 0.92, "shoot\nwindow",
            color="white", fontsize=8, va="top", alpha=0.75)

    ax.set_xlabel("Autonomous elapsed time (s)", color="white", fontsize=12)
    ax.set_ylabel("Cumulative energy (kJ)",      color="white", fontsize=12)
    ax.set_title("VACHE Q54 — Spindexer + Kicker Energy (Autonomous only)",
                 color="white", fontsize=13)
    ax.tick_params(colors="white")
    ax.set_xlim(0, t_elapsed[-1])
    ax.set_ylim(bottom=0)
    for spine in ax.spines.values():
        spine.set_edgecolor("#444466")
    ax.yaxis.set_major_formatter(ticker.FuncFormatter(lambda x, _: f"{x:.2f}"))

    handles, labels = ax.get_legend_handles_labels()
    ax.legend(handles[::-1], labels[::-1], loc="upper left", fontsize=10,
              facecolor="#1a1a2e", edgecolor="#444466", labelcolor="white")

    plt.tight_layout()
    out = Path(__file__).parent.parent / "doc" / "assets" / "energy_feeder_auto_q54.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    plt.savefig(out, dpi=150, facecolor=fig.get_facecolor())
    print(f"\nSaved -> {out}")
    plt.show()


if __name__ == "__main__":
    main()
