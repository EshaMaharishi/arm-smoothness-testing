#!/usr/bin/env python3
"""Plot MoveToJointPositions timing data.

Usage: plot_timings.py <input.csv> <output.png>

CSV columns: leg, idx_in_leg, start_unix_s, end_unix_s, duration_s
"""

import csv
import sys

import matplotlib.pyplot as plt


def main(csv_path: str, png_path: str) -> None:
    legs: list[str] = []
    starts: list[float] = []
    ends: list[float] = []
    durations: list[float] = []
    with open(csv_path) as f:
        reader = csv.DictReader(f)
        for row in reader:
            legs.append(row["leg"])
            starts.append(float(row["start_unix_s"]))
            ends.append(float(row["end_unix_s"]))
            durations.append(float(row["duration_s"]))

    n = len(durations)
    if n == 0:
        raise SystemExit("no timing rows in csv")

    # Inter-call gap = next.start - prev.end. First call's gap is undefined; record as 0.
    gaps = [0.0]
    for i in range(1, n):
        gaps.append(starts[i] - ends[i - 1])

    x = list(range(n))
    call_dur_ms = [d * 1000 for d in durations]
    gap_ms = [g * 1000 for g in gaps]

    fig, axes = plt.subplots(2, 1, sharex=True, figsize=(11, 6))

    axes[0].plot(x, call_dur_ms, marker=".", linewidth=0.8)
    axes[0].set_ylabel("Call duration (ms)")
    axes[0].set_title("MoveToJointPositions call duration")
    axes[0].grid(True, alpha=0.3)

    axes[1].plot(x, gap_ms, marker=".", linewidth=0.8, color="tab:orange")
    axes[1].set_ylabel("Gap to next start (ms)")
    axes[1].set_xlabel("Call index (global)")
    axes[1].set_title("Idle time between consecutive MoveToJointPositions calls")
    axes[1].grid(True, alpha=0.3)

    # Shade leg boundaries.
    prev_leg = legs[0]
    boundaries = [0]
    for i, leg in enumerate(legs[1:], start=1):
        if leg != prev_leg:
            boundaries.append(i)
            prev_leg = leg
    boundaries.append(n)
    leg_names = [legs[b] for b in boundaries[:-1]]
    shade_colors = ["#eef", "#fee", "#efe", "#ffe"]
    for ax in axes:
        for i in range(len(boundaries) - 1):
            ax.axvspan(boundaries[i], boundaries[i + 1], color=shade_colors[i % len(shade_colors)], alpha=0.5)
    # Label each leg span on the top axis.
    for i, name in enumerate(leg_names):
        mid = (boundaries[i] + boundaries[i + 1]) / 2
        axes[0].text(mid, axes[0].get_ylim()[1], name, ha="center", va="top", fontsize=8, alpha=0.7)

    fig.suptitle(f"Per-call timing — {n} calls across {len(leg_names)} legs")
    fig.tight_layout()
    fig.savefig(png_path, dpi=120)
    print(f"saved {png_path} (n={n})")


if __name__ == "__main__":
    if len(sys.argv) != 3:
        raise SystemExit("usage: plot_timings.py <input.csv> <output.png>")
    main(sys.argv[1], sys.argv[2])
