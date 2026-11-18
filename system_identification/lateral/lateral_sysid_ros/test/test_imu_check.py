import numpy as np

from lateral_sysid_ros.steady_state_from_bag import check_imu_frame

N = 1000
V_X = np.full(N, 3.0)
OMEGA = np.linspace(0.0, 2.0, N)
A_LAT = V_X * OMEGA
RNG = np.random.default_rng(0)
NOISE = RNG.normal(0, 0.2, (3, N))
FRAMES = ['base_link'] * N


def test_car_frame_passes():
    assert check_imu_frame(FRAMES, NOISE[0], A_LAT + NOISE[1], 9.81 + NOISE[2], V_X, OMEGA) is None


def test_namespaced_frame_passes():
    frames = ['ego_racecar/base_link'] * N
    assert check_imu_frame(frames, NOISE[0], A_LAT + NOISE[1], 9.81 + NOISE[2], V_X, OMEGA) is None


def test_wrong_frame_id_fails():
    assert 'frame_id' in check_imu_frame(['imu_link'] * N, NOISE[0], A_LAT, 9.81 + NOISE[2], V_X, OMEGA)


def test_swapped_axes_fail():
    # raw sensor mounted rotated by 90 degrees: lateral acceleration shows up on x
    assert 'axes' in check_imu_frame(FRAMES, A_LAT + NOISE[0], NOISE[1], 9.81 + NOISE[2], V_X, OMEGA)


def test_flipped_y_fails():
    assert 'axes' in check_imu_frame(FRAMES, NOISE[0], -A_LAT + NOISE[1], 9.81 + NOISE[2], V_X, OMEGA)


def test_upside_down_fails():
    assert 'a_z' in check_imu_frame(FRAMES, NOISE[0], A_LAT, -9.81 + NOISE[2], V_X, OMEGA)


def test_constant_axes():
    # a_x constant is fine, a_y constant is not
    assert check_imu_frame(FRAMES, np.zeros(N), A_LAT, np.full(N, 9.81), V_X, OMEGA) is None
    assert 'axes' in check_imu_frame(FRAMES, A_LAT, np.zeros(N), np.full(N, 9.81), V_X, OMEGA)
