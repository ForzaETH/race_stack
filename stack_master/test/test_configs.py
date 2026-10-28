"""Every config of this package that has a pydantic schema validates against it."""
from pathlib import Path

import pytest
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
