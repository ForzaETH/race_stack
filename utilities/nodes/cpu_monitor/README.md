# cpu_monitor

Lightweight per-ROS-node CPU/memory profiler for the ForzaETH race stack. Runs as a normal ROS 2 node, samples each ROS process via `psutil`, logs a sorted top-N table to the console every tick, and optionally writes a CSV for offline analysis. A companion script (`plot_cpu_log`) turns those CSVs into plots.

Built for the Jetson (6 cores), where overall CPU is near saturation and we need to attribute load to individual nodes (Cartographer, controller, state machine, etc.).

---

## How the profiling node works

1. **Discovery.** Every `pid_rescan_period_s` seconds (default 5 s) the node scans `/proc` for processes whose cmdline contains `--ros-args`. The ROS node name is taken from the `__node:=NAME` remap; if absent, the executable basename is used (covers `cartographer_node`, `controller`, etc., where node name == exec name). A PID is kept only if its derived label appears in `ros2 node list` (queried via the rclpy graph API — no shell-out).
2. **Sampling.** Every `1 / sample_rate_hz` seconds (default 1 Hz):
   - System-wide CPU is read via `psutil.cpu_percent()` (total, 0–100%) and `cpu_percent(percpu=True)` (per-core, 6 values on the Jetson).
   - For each cached PID, the node reads `cpu_percent(interval=None)` (per-core normalized — can exceed 100% for multi-threaded processes), `memory_info().rss`, and `num_threads()`.
   - Two CPU figures are reported:
     - `per-core%` — raw psutil value, 0–600% on the Jetson; matches `top` / `htop`.
     - `of-total%` — `per-core% / NCPU`; directly comparable to the system total and across nodes.
3. **Self-attribution.** The monitor includes its own PID in the table so you can see its overhead.
4. **Lightweight by design.** No subprocess forks, no per-tick topic publishing, `/proc` is walked only at the rescan cadence. Typical overhead is well under 1% of one core.

### Parameters

| Parameter             | Default | Meaning                                                |
|-----------------------|---------|--------------------------------------------------------|
| `sample_rate_hz`      | `1.0`   | How often to log a snapshot.                           |
| `pid_rescan_period_s` | `5.0`   | How often to re-walk `/proc` for new/dead nodes.       |
| `top_n`               | `10`    | Number of nodes shown in the per-tick log block.       |
| `csv_path`            | `""`    | If non-empty, append rows to this CSV. Empty disables. |

---

## How to use it

### 1. Install the dependency

`psutil` is not declared in the repo's Dockerfile. Inside the container:

```bash
python3 -c "import psutil; print(psutil.__version__)"   # check
pip install psutil                                       # if missing
```

### 2. Build

```bash
cd ~/ws
colcon build --packages-select cpu_monitor
source install/setup.bash
```

### 3. Launch with profiling on

The node is wired into `stack_master/launch/base_system_launch.xml` behind a `profile` flag (default `False` — zero overhead in normal races):

```bash
ros2 launch stack_master base_system_launch.xml \
    racecar_version:=NUC2 map_name:=hangar profile:=True
```

This logs a per-second block to the console and writes CSV to `/home/jetson/ws/log/cpu_<map_name>.csv` (which maps to `cache/humble/log/cpu_<map_name>.csv` on the host).

### 4. Standalone use

The node can also be launched directly:

```bash
ros2 launch cpu_monitor cpu_monitor_launch.xml \
    csv_path:=/home/jetson/ws/log/cpu_test.csv \
    sample_rate_hz:=1.0 top_n:=10
```

---

## What gets logged

### Console (every tick)

```
[cpu_monitor] system: 71.0% total | per-core: [ 82  65  90  60  71  58] | NCPU=6
  node                       per-core%   of-total%       RSS   thr
  cartographer_node              178.0%       29.7%     312MB    14
  controller                      41.0%        6.8%      52MB     3
  ...
```

### CSV columns

| Column               | Meaning                                                                  |
|----------------------|--------------------------------------------------------------------------|
| `t`                  | Seconds since the monitor started (monotonic).                           |
| `kind`               | `node` for per-process rows, `system` for the whole-system tick row.     |
| `label`              | ROS node name (or `TOTAL` for system rows).                              |
| `pid`                | Process ID (empty for system rows).                                      |
| `cpu_per_core_pct`   | psutil raw value, normalized to one core (0–600% on the Jetson).         |
| `cpu_of_total_pct`   | `cpu_per_core_pct / NCPU`. For system rows: whole-system total CPU%.     |
| `rss_bytes`          | Resident set size, bytes.                                                |
| `num_threads`        | Process thread count.                                                    |
| `core0` … `core5`    | Per-core CPU% (only populated on system rows).                           |

One `system` row plus one row per live ROS node is appended every sample. The file is opened in append mode — header is written only on first creation, so re-running with the same `csv_path` continues the previous file.

---

## Plotting: `plot_cpu_log`

A companion script in this package turns a CSV log into two PNGs and a numeric summary. Runs headless (matplotlib `Agg` backend) — no X server needed.

### Usage

```bash
ros2 run cpu_monitor plot_cpu_log /home/jetson/ws/log/cpu_hangar.csv
```

Optional flags:

| Flag             | Default     | Meaning                                                |
|------------------|-------------|--------------------------------------------------------|
| `--metric`       | `of_total`  | `of_total` (% of whole Jetson) or `per_core` (raw).    |
| `--top-n`        | `10`        | Keep the top-N nodes (by median CPU) in both plots.    |
| `--out-dir DIR`  | CSV's dir   | Where to save the PNGs.                                |

### Outputs

Saved alongside the CSV (e.g. `/home/jetson/ws/log/`):

1. **`<csv_stem>_box.png`** — horizontal box & whisker plot, one box per node, top-10 by median, heaviest on top. Mean is overlaid as a line marker. Title notes the top-10 filter, sample count, and run duration. Tells you on average over the run which nodes were heaviest.
2. **`<csv_stem>_timeseries.png`** — one line per top-10 node showing CPU% over time, plus a dashed black `system total` line for context. Makes it obvious whether a node's load is steady or spiky.
3. **stdout** — a per-node summary table with mean, std, median, max, and sample count. Std dev specifically isn't a box-plot primitive, so it's printed here.

Example:

```bash
$ ros2 run cpu_monitor plot_cpu_log /home/jetson/ws/log/cpu_hangar.csv
Wrote /home/jetson/ws/log/cpu_hangar_box.png
Wrote /home/jetson/ws/log/cpu_hangar_timeseries.png

node                                  mean       std    median       max       n
cartographer_node                    29.40%     4.12%    29.10%    41.30%      120
controller                            6.80%     0.95%     6.70%     9.20%      120
...
```
