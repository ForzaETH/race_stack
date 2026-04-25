#!/usr/bin/env python3
import os
import re
import time
import csv

import psutil
import rclpy
from rclpy.node import Node


_NODE_REMAP_RE = re.compile(r'__node:=([A-Za-z_][A-Za-z0-9_]*)')


def _label_from_cmdline(cmdline):
    """Return ROS node label for a process cmdline, or None if not a ROS 2 node."""
    if not cmdline:
        return None
    joined = ' '.join(cmdline)
    if '--ros-args' not in joined:
        return None
    m = _NODE_REMAP_RE.search(joined)
    if m:
        return m.group(1)
    # Fallback: executable basename. cmdline[0] is the interpreter for python
    # nodes (e.g. /usr/bin/python3), so prefer cmdline[1] when it looks like a path.
    exe = cmdline[0]
    if len(cmdline) > 1 and os.path.basename(exe).startswith('python'):
        exe = cmdline[1]
    return os.path.basename(exe)


class CpuMonitor(Node):
    def __init__(self):
        super().__init__('cpu_monitor')

        self.declare_parameter('sample_rate_hz', 1.0)
        self.declare_parameter('pid_rescan_period_s', 5.0)
        self.declare_parameter('top_n', 10)
        self.declare_parameter('csv_path', '')

        self.sample_rate_hz = float(self.get_parameter('sample_rate_hz').value)
        self.rescan_period_s = float(self.get_parameter('pid_rescan_period_s').value)
        self.top_n = int(self.get_parameter('top_n').value)
        self.csv_path = str(self.get_parameter('csv_path').value)

        self.ncpu = psutil.cpu_count() or 1
        self.own_pid = os.getpid()
        self.t0 = time.monotonic()
        self.procs = {}  # pid -> (psutil.Process, label)

        self.csv_file = None
        self.csv_writer = None
        if self.csv_path:
            new_file = not os.path.exists(self.csv_path)
            self.csv_file = open(self.csv_path, 'a', newline='')
            self.csv_writer = csv.writer(self.csv_file)
            if new_file:
                self.csv_writer.writerow([
                    't', 'kind', 'label', 'pid',
                    'cpu_per_core_pct', 'cpu_of_total_pct',
                    'rss_bytes', 'num_threads',
                    'core0', 'core1', 'core2', 'core3', 'core4', 'core5',
                ])

        # Prime psutil's system-wide cpu_percent baselines.
        psutil.cpu_percent(interval=None)
        psutil.cpu_percent(interval=None, percpu=True)

        self._rescan_pids()

        self.sample_timer = self.create_timer(1.0 / self.sample_rate_hz, self._on_sample)
        self.rescan_timer = self.create_timer(self.rescan_period_s, self._rescan_pids)

        self.get_logger().info(
            f'cpu_monitor started: NCPU={self.ncpu}, sample={self.sample_rate_hz}Hz, '
            f'rescan={self.rescan_period_s}s, top_n={self.top_n}, '
            f'csv={"on" if self.csv_path else "off"}'
        )

    def _rescan_pids(self):
        ros_names = {n for (n, _ns) in self.get_node_names_and_namespaces()}
        new_procs = {}
        for p in psutil.process_iter(['pid', 'cmdline']):
            try:
                pid = p.info['pid']
                if pid == self.own_pid:
                    continue
                label = _label_from_cmdline(p.info['cmdline'])
                if label is None:
                    continue
                if label not in ros_names and label != self.get_name():
                    # Keep only processes whose label appears in the ROS graph.
                    # (Our own node name self-filters via own_pid above.)
                    continue
                if pid in self.procs:
                    new_procs[pid] = self.procs[pid]
                else:
                    proc = psutil.Process(pid)
                    proc.cpu_percent(interval=None)  # prime
                    new_procs[pid] = (proc, label)
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                continue
        # Always include ourselves so users can observe the monitor's own overhead.
        if self.own_pid not in new_procs:
            try:
                me = psutil.Process(self.own_pid)
                me.cpu_percent(interval=None)
                new_procs[self.own_pid] = (me, self.get_name())
            except psutil.NoSuchProcess:
                pass
        self.procs = new_procs

    def _on_sample(self):
        sys_total = psutil.cpu_percent(interval=None)
        sys_per_core = psutil.cpu_percent(interval=None, percpu=True)
        t = time.monotonic() - self.t0

        rows = []
        dead = []
        for pid, (proc, label) in self.procs.items():
            try:
                cpu_raw = proc.cpu_percent(interval=None)
                rss = proc.memory_info().rss
                threads = proc.num_threads()
            except (psutil.NoSuchProcess, psutil.AccessDenied):
                dead.append(pid)
                continue
            rows.append((label, pid, cpu_raw, cpu_raw / self.ncpu, rss, threads))
        for pid in dead:
            self.procs.pop(pid, None)

        rows.sort(key=lambda r: r[2], reverse=True)

        per_core_str = ' '.join(f'{c:3.0f}' for c in sys_per_core)
        lines = [
            f'[cpu_monitor] system: {sys_total:5.1f}% total | per-core: [{per_core_str}] | NCPU={self.ncpu}',
            f'  {"node":<28}{"per-core%":>11}{"of-total%":>12}{"RSS":>10}{"thr":>6}',
        ]
        for label, _pid, cpu_raw, cpu_share, rss, threads in rows[:self.top_n]:
            lines.append(
                f'  {label[:28]:<28}{cpu_raw:>10.1f}%{cpu_share:>11.1f}%'
                f'{rss / (1024 * 1024):>8.0f}MB{threads:>6d}'
            )
        self.get_logger().info('\n'.join(lines))

        if self.csv_writer is not None:
            cores = list(sys_per_core) + [''] * (6 - len(sys_per_core))
            self.csv_writer.writerow([
                f'{t:.3f}', 'system', 'TOTAL', '',
                '', f'{sys_total:.2f}', '', '', *cores[:6],
            ])
            for label, pid, cpu_raw, cpu_share, rss, threads in rows:
                self.csv_writer.writerow([
                    f'{t:.3f}', 'node', label, pid,
                    f'{cpu_raw:.2f}', f'{cpu_share:.2f}',
                    rss, threads, '', '', '', '', '', '',
                ])
            self.csv_file.flush()

    def destroy_node(self):
        if self.csv_file is not None:
            try:
                self.csv_file.close()
            except Exception:
                pass
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CpuMonitor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
