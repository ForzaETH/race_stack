#!/usr/bin/env python3
"""Run a batch of benchmark races, one at a time, resuming wherever it stopped.

Each race gets its own folder holding race.jsonl, the planners' recordings and meta.json. A
folder with a meta.json is finished and is never run again, so the batch can be stopped with
Ctrl-C and restarted at any time. A race that dies gets MAX_TRIES attempts, then counts as failed.

Usage:  python3 bench_run.py bench_first_test.yaml [--pilot] [--dry-run] [--limit 10]
"""
import argparse
import json
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
ROOT = HERE.parents[3]                     # race_stack/
PLANNERS = {"spliner": ("spliner", 4), "gb": ("gb", 4), "mpc": ("mpc", 4),
            "ibr1": ("ibr", 1), "ibr4": ("ibr", 4)}     # name: (planner, game iterations)
MAX_TRIES = 3
STOP_GRACE = 8.0   # [s] time a launch gets to exit after SIGINT
STALL = 5.0        # [s] a started race whose log stops growing has lost its sim
HOLD = 3.0         # [s] the referee may run this far past t_max to finish a pass


def map_file(name):
    return ROOT / "stack_master" / "maps" / name / "global_waypoints.json"


def raceline_length(name):
    with open(map_file(name)) as f:
        return json.load(f)["global_traj_wpnts_iqp"]["wpnts"][-1]["s_m"]


def list_races(cfg, pilot):
    """Every race of the batch. The spawns are the same for each pairing, so pairings can be
    compared race by race. Attackers run in the order listed, so the last one is what gets cut
    if time runs out."""
    races = []
    for attacker in cfg["attackers"]:
        for name in cfg["pilot_maps"] if pilot else cfg["maps"]:
            L = raceline_length(name)
            spots = [i * L / cfg["n_positions"] for i in range(cfg["n_positions"])]
            spots = spots[::cfg["every"]][:cfg["use_positions"]]
            spawns = [(k, s, gap) for k, s in enumerate(spots) for gap in cfg["gaps_s"]]
            if pilot:
                spawns = spawns[::10]             # three spots, one of each gap
            for defender in cfg["defenders"]:
                for k, s, gap in spawns:
                    races.append(dict(map=name, attacker=attacker, defender=defender, spot=k,
                                      s=s, gap_s=gap, gap_m=gap * cfg["defender_speed"]))
    return races


def launch_args(race, cfg, out):
    ego, ego_n = PLANNERS[race["attacker"]]
    opp, opp_n = PLANNERS[race["defender"]]
    return [f"map_name:={race['map']}", f"ego_planner:={ego}", f"opp_planner:={opp}",
            f"ego_n_game_iterations:={ego_n}", f"opp_n_game_iterations:={opp_n}",
            "ego_speed_scale:=1.0", "opp_speed_scale:=1.0",
            f"ego_speed_limit:={cfg['attacker_speed']}", f"opp_speed_limit:={cfg['defender_speed']}",
            f"flat_speed:={max(cfg['attacker_speed'], cfg['defender_speed'])}",
            f"ego_s:={race['s']:.3f}", f"opp_s:={race['s'] + race['gap_m']:.3f}",
            f"ego_record_path:={out / 'ego.rec.jsonl'}", f"opp_record_path:={out / 'opp.rec.jsonl'}",
            "opponent_source:=gt", "headless:=true",
            f"time_scale:={cfg.get('time_scale', 1.0)}"]
            


def start(cmd, log):
    # Its own process group, so one signal reaches the launch and every node under it. SIGINT is
    # reset because a batch started with nohup or & would pass "ignore it" down to all of them.
    return subprocess.Popen(cmd, stdout=log, stderr=subprocess.STDOUT, start_new_session=True,
                            preexec_fn=lambda: signal.signal(signal.SIGINT, signal.SIG_DFL))


def stop(proc):
    try:
        os.killpg(proc.pid, signal.SIGINT)
        proc.wait(STOP_GRACE)
    except subprocess.TimeoutExpired:
        pass
    except ProcessLookupError:
        return
    try:
        os.killpg(proc.pid, signal.SIGKILL)       # whatever ignored the polite request
    except ProcessLookupError:
        pass
    proc.wait()


def last_row(log):
    """The referee's verdict. Read from the log, not its exit code: zenoh can abort the
    process on the way out, after the outcome has been written."""
    if not log.exists():
        return None
    lines = log.read_text().strip().split("\n")
    try:
        row = json.loads(lines[-1])
    except json.JSONDecodeError:
        return None
    return row if "outcome" in row else None


def run_race(race, cfg, out, start_timeout):
    """One attempt. Returns the outcome row, or None if the race died."""
    log = out / "race.jsonl"
    log.unlink(missing_ok=True)
    referee = start([sys.executable, str(HERE / "bench_referee.py"), str(log),
                        str(map_file(race["map"])), "--t-max", str(cfg["t_max"]),
                     "--time-scale", str(cfg.get("time_scale", 1.0))],
                    subprocess.DEVNULL)
    with open(out / "launch.log", "w") as f:
        launch = start(["ros2", "launch", "stack_master", "sim_two_car_launch.xml",
                        *launch_args(race, cfg, out)], f)
    t0 = time.time()
    try:
        while referee.poll() is None and launch.poll() is None:
            now = time.time()
            started = log.exists() and log.stat().st_size > 0
            if not started and now - t0 > start_timeout:        # e.g. a planner died at launch
                break
            if started and now - log.stat().st_mtime > STALL:   # e.g. the sim crashed
                break
            time.sleep(0.2)
    finally:
        stop(referee)
        stop(launch)
    return last_row(log)


def finish(out, race, row, tries, seconds):
    outcome = row["outcome"] if row and row["outcome"] != "interrupted" else "failed"
    meta = dict(race, outcome=outcome, tries=tries, seconds=round(seconds, 1),
                t_end=row.get("t_end") if row else None,
                min_dist=row.get("min_dist") if row else None,
                engaged=row.get("engaged") if row else None)
    tmp = out / "meta.tmp"                        # a crash here cannot leave half a meta.json
    tmp.write_text(json.dumps(meta))
    tmp.rename(out / "meta.json")
    if outcome != "failed":
        (out / "launch.log").unlink(missing_ok=True)   # kept for failed races only
    return outcome


def main():
    signal.signal(signal.SIGINT, signal.default_int_handler)   # Ctrl-C works, even under nohup
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("config", help="yaml file listing the batch")
    ap.add_argument("--pilot", action="store_true", help="only the small pilot subset")
    ap.add_argument("--dry-run", action="store_true", help="list what would run, run nothing")
    ap.add_argument("--limit", type=int, help="stop after this many races")
    ap.add_argument("--start-timeout", type=float, default=45.0,
                    help="seconds for the cars to start moving after launch")
    args = ap.parse_args()

    with open(args.config) as f:
        cfg = yaml.safe_load(f)
    folder = "pilot" if args.pilot else Path(args.config).stem
    results = ROOT / "bench_results" / folder
    races = list_races(cfg, args.pilot)

    def out_dir(r):
        return (results / r["map"] / f"{r['attacker']}_vs_{r['defender']}"
                / f"spot{r['spot']}_gap{r['gap_s']}")

    todo = [r for r in races if not (out_dir(r) / "meta.json").exists()]
    print(f"{len(races)} races, {len(races) - len(todo)} done, {len(todo)} to run -> {results}")
    if args.dry_run:
        return
    t_batch = time.time()
    for k, race in enumerate(todo[:args.limit]):
        out = out_dir(race)
        out.mkdir(parents=True, exist_ok=True)
        t0, row = time.time(), None
        for tries in range(1, MAX_TRIES + 1):
            row = run_race(race, cfg, out, args.start_timeout)
            if row and row["outcome"] != "interrupted":
                break
            time.sleep(2.0)
        outcome = finish(out, race, row, tries, time.time() - t0)
        per_race = (time.time() - t_batch) / (k + 1)
        left = (len(todo) - k - 1) * per_race / 3600
        print(f"[{k + 1}/{len(todo)}] {out.relative_to(results)}: {outcome} "
              f"(try {tries}, {time.time() - t0:.0f} s; about {left:.1f} h left)", flush=True)
        time.sleep(2.0)


if __name__ == "__main__":
    main()
