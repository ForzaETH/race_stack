import numpy as np
from on_track_sysid import PacejkaParams, VehicleParams, generate_lut

from steering_lookup.lookup_steer_angle import LookupSteerAngle

VEHICLE = VehicleParams(m=3.39, Iz=0.05797, lf=0.168, lr=0.167, h_cg=0.014)
TIRES = PacejkaParams(Bf=7.1084, Cf=0.683, Df=2.0, Ef=0.445, Br=4.9599, Cr=1.5278, Dr=1.1269, Er=1.1)


def test_lut_round_trip():
    table = generate_lut(VEHICLE, TIRES, steers=np.linspace(0, 0.4, 41), vels=[1.0, 3.0, 5.0])
    lookup = LookupSteerAngle.__new__(LookupSteerAngle)
    lookup.lu = table
    for j, vel in enumerate(table[0, 1:], start=1):
        for i in range(2, table.shape[0]):
            a_lat = table[i, j]
            if np.isnan(a_lat):
                break
            assert abs(lookup.lookup_steer_angle(a_lat, vel) - table[i, 0]) < 1e-9
            if i == 2:
                continue  # next to the first row the lookup does not interpolate
            # between two rows: linear interpolation, sign follows the acceleration
            mid = (table[i - 1, j] + a_lat) / 2
            assert abs(lookup.lookup_steer_angle(-mid, vel) + (table[i - 1, 0] + table[i, 0]) / 2) < 1e-9
