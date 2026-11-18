"""Glue between forza_config_ros (stack_master/config) and the on_track_sysid library."""
from dataclasses import asdict
from datetime import datetime
from pathlib import Path

import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from on_track_sysid import PacejkaParams, VehicleParams, generate_lut, pacejka_formula  # noqa: E402
from on_track_sysid.pacejka import axle_loads  # noqa: E402
from forza_config import PacejkaTireConfig  # noqa: E402
from forza_config_ros import (NotExistingFloor, get_stack_master_path, load_car_config_ros,  # noqa: E402
                              load_pacejka_tire_config_ros, save_pacejka_tire_config_ros, save_steering_lut_ros)

# Start point when a floor has no identified tires yet; convergence does not hinge on it
GENERIC_TIRES = PacejkaParams(Bf=5.0, Cf=1.0, Df=1.5, Ef=0.5, Br=5.0, Cr=1.0, Dr=1.5, Er=0.5, friction_coeff=1.0)


def load_vehicle(racecar_version: str) -> VehicleParams:
    car = load_car_config_ros(racecar_version)
    return VehicleParams(**car.model_dump(include={'m', 'Iz', 'lf', 'lr', 'h_cg'}))


def load_tires(racecar_version: str, floor: str, log=print) -> PacejkaParams:
    try:
        tires = load_pacejka_tire_config_ros(racecar_version, floor)
    except (NotExistingFloor, FileNotFoundError):
        log(f"No tires identified yet for {racecar_version} on {floor}, starting from a generic model")
        return GENERIC_TIRES
    return PacejkaParams(**tires.model_dump(exclude={'floor'}))


def data_dir(racecar_version: str, floor: str) -> Path:
    path = get_stack_master_path(f'config/{racecar_version}/pacejka/{floor}/data')
    path.mkdir(parents=True, exist_ok=True)
    return path


def save_raw_data(data, racecar_version: str, floor: str, name: str, header: str = 'v_x,v_y,omega,delta') -> Path:
    path = data_dir(racecar_version, floor) / f'{datetime.now():%Y%m%d_%H%M%S}_{name}.csv'
    np.savetxt(path, data, delimiter=',', header=header, comments='')
    return path


def save_results(tires: PacejkaParams, vehicle: VehicleParams, racecar_version: str, floor: str,
                 update_latest: bool = True, log=print):
    """Saves the tire config, then generates and saves the steering LUT from it."""
    path = save_pacejka_tire_config_ros(PacejkaTireConfig(**asdict(tires), floor=floor),
                                        racecar_version, floor, update_latest)
    log(f"Saved tires to {path}")
    log("Generating the steering LUT...")
    path = save_steering_lut_ros(generate_lut(vehicle, tires), racecar_version, floor, update_latest)
    log(f"Saved steering LUT to {path}")


def plot_tires(path: Path, vehicle: VehicleParams, tires: PacejkaParams, reference: PacejkaParams = None,
               samples=None):
    """Plots the identified front and rear tire curves, optionally against a reference model
    and (alpha_f, F_yf, alpha_r, F_yr) samples."""
    alpha = np.linspace(-0.4, 0.4, 200)
    F_zf, F_zr = axle_loads(vehicle)
    fig, axes = plt.subplots(2, 1, figsize=(8, 6), sharex=True)
    for ax, coeffs, ref, F_z, name, idx in ((axes[0], tires.front, reference and reference.front, F_zf, 'Front', 0),
                                            (axes[1], tires.rear, reference and reference.rear, F_zr, 'Rear', 2)):
        if samples is not None:
            ax.scatter(samples[idx], samples[idx + 1], s=4, alpha=0.2, color='tab:green', label='samples')
        if ref is not None:
            ax.plot(alpha, pacejka_formula(ref, alpha, F_z, reference.friction_coeff), '--', color='gray',
                    label='initial')
        ax.plot(alpha, pacejka_formula(coeffs, alpha, F_z, tires.friction_coeff), color='tab:blue', label='identified')
        ax.set_title(f'{name} tires')
        ax.set_ylabel(r'$F_y$ [N]')
        ax.grid()
        ax.legend(loc='best')
    axes[1].set_xlabel(r'$\alpha$ [rad]')
    fig.tight_layout()
    fig.savefig(path)
    plt.close(fig)
