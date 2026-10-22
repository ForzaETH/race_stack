from pathlib import Path

import numpy as np
import pytest
import yaml

import forza_config_ros.config as cfg

TIRE = dict(Bf=4.142, Br=4.8721, Cf=0.4375, Cr=2.1651, Df=2.6476, Dr=1.057, Ef=0.2332, Er=1.692,
            floor='dubi', friction_coeff=1.0)


@pytest.fixture
def tree(tmp_path, monkeypatch):
    monkeypatch.setattr(cfg, 'get_stack_master_path', lambda subpath='': tmp_path / subpath)
    return tmp_path


def test_pacejka_save_load_and_missing_floor(tree):
    cfg.save_pacejka_tire_config_ros(cfg.PacejkaTireConfig(**TIRE), 'NUC5', 'dubi')
    assert (tree / 'config/NUC5/pacejka/dubi/default.yaml').is_symlink()
    assert cfg.load_pacejka_tire_config_ros('NUC5', 'dubi').Bf == pytest.approx(4.142)
    with pytest.raises(cfg.NotExistingFloor):
        cfg.load_pacejka_tire_config_ros('NUC5', 'hangar')
    with pytest.raises(FileNotFoundError):
        cfg.load_pacejka_tire_config_ros('NUC7', 'dubi')


def test_lut_round_trip(tree):
    table = np.array([[0.0, 1.0, 2.0], [0.1, 0.5, np.nan]])
    cfg.save_steering_lut_ros(table, 'NUC5', 'dubi')
    np.testing.assert_array_equal(cfg.load_steering_lut_ros('NUC5', 'dubi'), table)


def test_real_tree_cars_load():
    """Every car_model.yaml and pacejka default in this checkout validates."""
    config = Path(__file__).resolve().parents[4] / 'stack_master/config'
    cars = sorted(p.parent.name for p in config.glob('*/car_model.yaml'))
    assert cars
    for path in config.glob('*/car_model.yaml'):
        cfg.load_car_config(path)
    for path in config.glob('*/pacejka/*/default.yaml'):
        cfg.load_pacejka_tire_config(path)


def test_load_model_ros_any_schema(tree):
    from pydantic import BaseModel, ConfigDict

    class Gains(BaseModel):
        model_config = ConfigDict(extra='forbid')
        kp: float

    folder = tree / 'config/NUC5/mpc'
    folder.mkdir(parents=True)
    (folder / 'gains.yaml').write_text('kp: 2.0\n')
    assert cfg.load_model_ros(Gains, 'NUC5', 'mpc', 'gains.yaml').kp == 2.0
