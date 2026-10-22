"""Regenerates the steering LUT of a car and floor from its current pacejka/<floor>/default.yaml."""
import argparse

from on_track_sysid import generate_lut
from forza_config_ros import save_steering_lut_ros

from lateral_sysid_ros.config_io import load_tires, load_vehicle


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--car', required=True, help='racecar_version, e.g. NUC6')
    parser.add_argument('--floor', required=True)
    parser.add_argument('--no-update-latest', action='store_true', help='do not point default.csv at the new LUT')
    args = parser.parse_args()

    tires = load_tires(args.car, args.floor, lambda msg: parser.error(msg))
    path = save_steering_lut_ros(generate_lut(load_vehicle(args.car), tires), args.car, args.floor,
                                 not args.no_update_latest)
    print(f"Saved steering LUT to {path}")
