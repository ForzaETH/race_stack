#!/usr/bin/env python3
"""Offline plotter for cpu_monitor CSV logs.

Usage:
    ros2 run cpu_monitor plot_cpu_log <csv_path> [--metric of_total|per_core]
                                                 [--top-n 10] [--out-dir DIR]
"""
import argparse
import csv
import os
from collections import defaultdict

import matplotlib
matplotlib.use('Agg')  # headless: no display needed
import matplotlib.pyplot as plt
import numpy as np


METRIC_COL = {
    'of_total': 'cpu_of_total_pct',
    'per_core': 'cpu_per_core_pct',
}
METRIC_LABEL = {
    'of_total': '% of total Jetson capacity',
    'per_core': '% of one core (psutil raw)',
}


def parse_csv(path, metric):
    col = METRIC_COL[metric]
    node_data = defaultdict(list)   # label -> list[(t, cpu)]
    sys_data = []                   # list[(t, cpu_total)]
    with open(path, 'r', newline='') as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                t = float(row['t'])
            except (TypeError, ValueError):
                continue
            kind = row.get('kind', '')
            if kind == 'node':
                label = row['label']
                try:
                    cpu = float(row[col])
                except (TypeError, ValueError):
                    continue
                node_data[label].append((t, cpu))
            elif kind == 'system':
                # System TOTAL row stores its value in cpu_of_total_pct.
                try:
                    cpu = float(row['cpu_of_total_pct'])
                except (TypeError, ValueError):
                    continue
                sys_data.append((t, cpu))
    return node_data, sys_data


def pick_top_n(node_data, n):
    """Return labels of top-n nodes by median CPU, sorted desc."""
    medians = {label: float(np.median([c for _t, c in pts]))
               for label, pts in node_data.items() if pts}
    return sorted(medians, key=medians.get, reverse=True)[:n]


def plot_box(node_data, top_labels, metric, n_samples, duration_s, out_path):
    # Horizontal box plot, heaviest node on top.
    values = [[c for _t, c in node_data[label]] for label in top_labels]
    labels_top_first = list(reversed(top_labels))
    values_top_first = list(reversed(values))

    fig, ax = plt.subplots(figsize=(10, max(4, 0.4 * len(top_labels) + 2)))
    ax.boxplot(values_top_first, labels=labels_top_first, vert=False,
               showmeans=True, meanline=True)
    ax.set_xlabel(f'CPU usage ({METRIC_LABEL[metric]})')
    ax.set_title(
        f'Per-node CPU distribution — top {len(top_labels)} by median\n'
        f'{n_samples} samples over {duration_s:.1f}s'
    )
    ax.grid(axis='x', linestyle=':', alpha=0.5)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def plot_timeseries(node_data, sys_data, top_labels, metric, out_path):
    fig, ax = plt.subplots(figsize=(12, 6))
    for label in top_labels:
        pts = sorted(node_data[label])
        ts = [p[0] for p in pts]
        cs = [p[1] for p in pts]
        ax.plot(ts, cs, label=label, linewidth=1.2)
    if sys_data and metric == 'of_total':
        sys_sorted = sorted(sys_data)
        ax.plot([p[0] for p in sys_sorted], [p[1] for p in sys_sorted],
                label='system total', linewidth=2.0, linestyle='--', color='black')
    ax.set_xlabel('Time (s)')
    ax.set_ylabel(f'CPU usage ({METRIC_LABEL[metric]})')
    ax.set_title(f'CPU usage over time — top {len(top_labels)} nodes by median')
    ax.grid(linestyle=':', alpha=0.5)
    ax.legend(loc='upper right', fontsize='small', ncol=2)
    fig.tight_layout()
    fig.savefig(out_path, dpi=120)
    plt.close(fig)


def main():
    ap = argparse.ArgumentParser(description='Plot cpu_monitor CSV logs.')
    ap.add_argument('csv_path', help='Path to cpu_monitor CSV log')
    ap.add_argument('--metric', choices=list(METRIC_COL), default='of_total')
    ap.add_argument('--top-n', type=int, default=10)
    ap.add_argument('--out-dir', default=None,
                    help='Where to save PNGs (default: same dir as CSV)')
    args = ap.parse_args()

    if not os.path.isfile(args.csv_path):
        raise SystemExit(f'CSV not found: {args.csv_path}')

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(args.csv_path))
    os.makedirs(out_dir, exist_ok=True)
    stem = os.path.splitext(os.path.basename(args.csv_path))[0]
    box_path = os.path.join(out_dir, f'{stem}_box.png')
    ts_path = os.path.join(out_dir, f'{stem}_timeseries.png')

    node_data, sys_data = parse_csv(args.csv_path, args.metric)
    if not node_data:
        raise SystemExit('No node rows found in CSV.')

    top_labels = pick_top_n(node_data, args.top_n)
    n_samples = sum(len(v) for v in node_data.values())
    all_times = [t for pts in node_data.values() for t, _c in pts]
    duration_s = max(all_times) - min(all_times) if all_times else 0.0

    plot_box(node_data, top_labels, args.metric, n_samples, duration_s, box_path)
    plot_timeseries(node_data, sys_data, top_labels, args.metric, ts_path)

    # Print a small numeric summary so users see std dev too.
    print(f'Wrote {box_path}')
    print(f'Wrote {ts_path}')
    print()
    print(f'{"node":<32}{"mean":>10}{"std":>10}{"median":>10}{"max":>10}{"n":>8}')
    for label in top_labels:
        cs = np.array([c for _t, c in node_data[label]])
        print(f'{label[:32]:<32}{cs.mean():>9.2f}%{cs.std():>9.2f}%'
              f'{np.median(cs):>9.2f}%{cs.max():>9.2f}%{len(cs):>8d}')


if __name__ == '__main__':
    main()
