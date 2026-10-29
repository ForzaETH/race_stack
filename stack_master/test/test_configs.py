"""Every config of this package that has a pydantic schema validates against it."""
from pathlib import Path

import pytest
import yaml
from forza_config import CarConfig, PacejkaTireConfig, TrailingConfig, load_model

mpc_config = pytest.importorskip('forzaeth_mpc.config')

CONFIG_DIR = Path(__file__).resolve().parents[1] / 'config'

# glob under config/ -> schema
SCHEMAS = {
    '*/car_model.yaml': CarConfig,
    '*/pacejka/**/*.yaml': PacejkaTireConfig,
    '*/trailing_params.yaml': TrailingConfig,
    '*/kinematic_mpc_params.yaml': mpc_config.KMPCConfig,
    '*/kinematic_mpc_dyn_params.yaml': mpc_config.KMPCConfigDyn,
    '*/single_track_mpc_params.yaml': mpc_config.STMPCConfig,
    '*/single_track_mpc_dyn_params.yaml': mpc_config.STMPCConfigDyn,
    '*/lb_mpc_params.yaml': mpc_config.LBMPCConfig,
    '*/lb_mpc_dyn_params.yaml': mpc_config.LBMPCConfigDyn,
    '*/time_optimal_mpc_params.yaml': mpc_config.TOMPCConfig,
    '*/time_optimal_mpc_dyn_params.yaml': mpc_config.TOMPCConfigDyn,
}

CASES = [(path, schema) for pattern, schema in SCHEMAS.items() for path in sorted(CONFIG_DIR.glob(pattern))]


def test_configs_found():
    assert CASES, f'no config with a schema under {CONFIG_DIR}'


@pytest.mark.parametrize('path, schema', CASES, ids=[str(path.relative_to(CONFIG_DIR)) for path, _ in CASES])
def test_config_validates(path, schema):
    load_model(schema, path)


@pytest.mark.parametrize('path', sorted(CONFIG_DIR.glob('*/car_model.yaml')), ids=lambda p: p.parent.name)
def test_wheelbase_is_lf_plus_lr(path):
    car = load_model(CarConfig, path)
    assert car.wheelbase == pytest.approx(car.lf + car.lr)


# the MPC must not plan steering angles that actuation_manager clips away
MPC_STEERING = [path for path in sorted(CONFIG_DIR.glob('*/*_mpc_params.yaml')) if (path.parent / 'vesc.yaml').exists()]


@pytest.mark.parametrize('path', MPC_STEERING, ids=[str(path.relative_to(CONFIG_DIR)) for path in MPC_STEERING])
def test_mpc_steering_matches_vesc(path):
    with open(path.parent / 'vesc.yaml') as f:
        limits = yaml.safe_load(f)['actuation_manager']['ros__parameters']
    with open(path) as f:
        mpc = yaml.safe_load(f)
    assert (mpc['delta_min'], mpc['delta_max']) == pytest.approx((limits['steering_min'], limits['steering_max']))
