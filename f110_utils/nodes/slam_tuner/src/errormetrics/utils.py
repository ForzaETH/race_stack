import numpy as np


def ang_btw_vec(v1, v2):
    ''' Calculate angle between 2d vectors using dot product formulation '''

    num = np.dot(v1, v2)
    den = np.linalg.norm(v1) * np.linalg.norm(v2)

    if np.allclose(den, 0.0):
        return 0.0

    # Clamp to a valid region
    theta = max(min(1.0, num/den), -1.0)

    return np.arccos(theta)


def transform_stuff(mo, obs):
    res = []

    for i in range(min(len(obs), len(mo))):
        mo_x, mo_y, mo_theta, _ = mo[i]
        t_mo = np.array([[np.cos(mo_theta), -np.sin(mo_theta), mo_x],
                         [np.sin(mo_theta),  np.cos(mo_theta), mo_y],
                         [0,                 0,                1]])

        obs_x, obs_y, obs_theta, t = obs[i]
        t_obs = np.array([[np.cos(obs_theta), -np.sin(obs_theta), obs_x],
                          [np.sin(obs_theta),  np.cos(obs_theta), obs_y],
                          [0,                  0,                 1]])

        t_res = t_mo @ t_obs    # transformation matrix shenanigans
        theta = np.arctan2(t_res[1, 0], t_res[0, 0])
        x = t_res[0, 2]
        y = t_res[1, 2]

        res.append((x, y, theta, t))

    return res


def trim_start_end_idx(data, start_time, end_time):
    '''
    Trims a dataset to find indexes between start_time and end_time

    This helps to temporally align our dataset because we always replay the same source ROSBAG.
    '''
    assert start_time >= 0.0
    assert start_time < end_time

    start = data[0][3]
    has_started = False
    has_ended = False
    start_idx = 0
    end_idx = -1

    for i, (_, _, _, t) in enumerate(data):
        if t-start > start_time and has_started is False:
            has_started = True
            start_idx = i

        if t-start > end_time and has_ended is False:
            has_ended = True
            end_idx = i-1

            break

    # print(start_idx, end_idx)
    return data[start_idx:end_idx]
