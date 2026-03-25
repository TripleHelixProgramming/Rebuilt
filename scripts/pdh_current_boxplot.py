"""
Box & whisker plot of total PDH current draw (A) for each usable VACHE match log.
Stats are pre-computed from /RealOutputs/PDH/TotalCurrentAmps, filtered to match time
(auto start → teleop end). Q13 excluded (no teleop detected). Q27 noted as anomalous.
"""

import matplotlib.pyplot as plt
import matplotlib.patches as mpatches
import numpy as np

# Pre-computed stats from wpilog server (min, Q1, median, Q3, max, mean)
# Format: label, min, Q1, median, Q3, max, mean, match_type
matches = [
    ("Q4",  2,  58, 100, 140, 266, 101.7, "qual"),
    ("Q10", 2,  78, 106, 134, 226, 105.6, "qual"),
    ("Q20", 2,  52,  90, 130, 292,  94.7, "qual"),
    ("Q23", 2,  66,  98, 134, 292, 101.4, "qual"),
    ("Q27", 0,  22,  28,  74, 246,  53.9, "qual"),  # anomalous — likely disabled/early end
    ("Q32", 2,  58,  86, 114, 250,  87.9, "qual"),
    ("Q36", 2,  60,  96, 136, 290, 100.1, "qual"),
    ("Q45", 2,  50,  88, 132, 266,  93.8, "qual"),
    ("Q48", 2,  52,  94, 138, 264,  98.4, "qual"),
    ("Q54", 2,  72, 108, 148, 266, 110.5, "qual"),
    ("Q58", 2,  46,  94, 138, 256,  97.2, "qual"),
    ("E4",  2,  58, 104, 146, 260, 104.3, "elim"),
    ("E8",  2,  64,  96, 132, 276,  99.2, "elim"),
    ("E10", 2,  56,  94, 134, 258,  97.0, "elim"),
]

labels  = [m[0] for m in matches]
mins    = [m[1] for m in matches]
q1s     = [m[2] for m in matches]
medians = [m[3] for m in matches]
q3s     = [m[4] for m in matches]
maxs    = [m[5] for m in matches]
means   = [m[6] for m in matches]
types   = [m[7] for m in matches]

# Build bxp stats dicts
bxp_stats = []
for i in range(len(matches)):
    bxp_stats.append({
        "med":    medians[i],
        "q1":     q1s[i],
        "q3":     q3s[i],
        "whislo": mins[i],
        "whishi": maxs[i],
        "mean":   means[i],
        "label":  labels[i],
        "fliers": [],
    })

# Colors
QUAL_COLOR = "#4C9BE8"
ELIM_COLOR = "#E87C4C"
ANOMALY_COLOR = "#AAAAAA"

colors = []
for m in matches:
    if m[0] == "Q27":
        colors.append(ANOMALY_COLOR)
    elif m[7] == "qual":
        colors.append(QUAL_COLOR)
    else:
        colors.append(ELIM_COLOR)

fig, ax = plt.subplots(figsize=(14, 6))

bp = ax.bxp(
    bxp_stats,
    showfliers=False,
    showmeans=True,
    meanline=False,
    patch_artist=True,
    widths=0.6,
    meanprops=dict(marker="D", markeredgecolor="white", markerfacecolor="white", markersize=5, zorder=5),
)

# Apply colors
for patch, color in zip(bp["boxes"], colors):
    patch.set_facecolor(color)
    patch.set_alpha(0.85)

for median_line in bp["medians"]:
    median_line.set_color("white")
    median_line.set_linewidth(2)

for whisker in bp["whiskers"]:
    whisker.set_color("#555555")
    whisker.set_linewidth(1.2)

for cap in bp["caps"]:
    cap.set_color("#555555")
    cap.set_linewidth(1.5)

# Shading between elim matches
elim_indices = [i + 1 for i, m in enumerate(matches) if m[7] == "elim"]
if elim_indices:
    ax.axvspan(min(elim_indices) - 0.5, max(elim_indices) + 0.5, alpha=0.06, color=ELIM_COLOR, zorder=0)

# Reference lines
ax.axhline(200, color="red", linestyle="--", linewidth=1, alpha=0.5, label="200 A ref")
ax.axhline(100, color="gray", linestyle=":", linewidth=1, alpha=0.4)

# Annotate Q27
q27_idx = labels.index("Q27") + 1
ax.annotate(
    "anomalous\n(low current)",
    xy=(q27_idx, 28),
    xytext=(q27_idx + 0.8, 70),
    arrowprops=dict(arrowstyle="->", color="#888888"),
    fontsize=8,
    color="#666666",
)

# Formatting
ax.set_xlabel("Match", fontsize=12)
ax.set_ylabel("Total PDH Current (A)", fontsize=12)
ax.set_title("PDH Total Current Draw — VACHE Match Logs\n(auto + teleop, /RealOutputs/PDH/TotalCurrentAmps)", fontsize=13)
ax.set_ylim(-5, 320)
ax.yaxis.grid(True, linestyle="--", alpha=0.4)
ax.set_axisbelow(True)

# Legend
qual_patch  = mpatches.Patch(color=QUAL_COLOR, alpha=0.85, label="Qualification")
elim_patch  = mpatches.Patch(color=ELIM_COLOR, alpha=0.85, label="Elimination")
anom_patch  = mpatches.Patch(color=ANOMALY_COLOR, alpha=0.85, label="Q27 (anomalous)")
mean_marker = plt.Line2D([0], [0], marker="D", color="w", markerfacecolor="#333333",
                         markersize=6, label="Mean")
ax.legend(handles=[qual_patch, elim_patch, anom_patch, mean_marker],
          loc="upper left", fontsize=9, framealpha=0.8)

plt.tight_layout()
plt.savefig("pdh_current_boxplot.png", dpi=150, bbox_inches="tight")
print("Saved pdh_current_boxplot.png")
plt.show()
