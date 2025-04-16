import os
import rospkg
from datetime import datetime
from scipy.integrate import odeint
import fire
import numpy as np
import matplotlib.pyplot as plt
from pbl_config import CarConfig, PacejkaTireConfig, load_car_config_ros, load_pacejka_tire_config_ros
from helpers.vehicle_dynamics import vehicle_dynamics

# Simulation parameters
SIMULATION_DURATION = 2.0  # seconds
SIMULATION_DT = 0.01  # seconds
PLOT_LOOKUP = True

# Lookup parameters
START_STEER = 0.0  # rad
STEER_FINE_END = 0.1  # rad
FINE_STEP_SIZE = 0.0033  # rad
END_STEER = 0.4  # rad
COARSE_STEP_SIZE = 0.01  # rad
START_VEL = 0.5  # m/s
END_VEL = 7.0  # m/s
VEL_STEP_SIZE = 0.1  # m/s

class Simulator:
    def __init__(self,
                 racecar_version: str,
                 floor: str):
        self.car_config: CarConfig = load_car_config_ros(racecar_version)
        self.pacejka_config: PacejkaTireConfig = load_pacejka_tire_config_ros(racecar_version, floor)
        self.sol = None

    def func_ST(self, x, t, u):
        """Wrapper to the vehicle dynamics function to be used with scipy.integrate.odeint"""
        f = vehicle_dynamics(x, u, self.car_config, self.pacejka_config)
        return f

    def run_simulation(self, initialState, u,
                       duration=SIMULATION_DURATION, dt=SIMULATION_DT):
        t = np.arange(0, duration, dt)
        self.sol = odeint(self.func_ST, initialState, t, args=(u,))
        return self.sol


class LookupGenerator:
    def __init__(self, racecar_version: str, floor: str, update_latest: bool= True):
        self.sim = Simulator(racecar_version, floor)
        self.racecar_version = racecar_version
        self.floor = floor
        self.update_latest = update_latest
        self.lookup_table = None

    def run_generator(self):
        self.generate_lookup()
        self.find_upper_limits()
        if PLOT_LOOKUP:
            self.plot_lookup()
        self.save_lookup()

    def generate_lookup(self):
        fine_steers = np.linspace(START_STEER, STEER_FINE_END, int(
            (STEER_FINE_END - START_STEER) / FINE_STEP_SIZE), endpoint=False)
        coarse_steers = np.linspace(STEER_FINE_END, END_STEER, int((END_STEER - STEER_FINE_END) / COARSE_STEP_SIZE))
        steers = np.concatenate((fine_steers, coarse_steers))
        vels = np.linspace(START_VEL, END_VEL, int((END_VEL - START_VEL) / VEL_STEP_SIZE))
        n_steps_steer = len(steers)
        n_steps_vel = len(vels)

        self.lookup_table = np.empty([n_steps_steer + 1, n_steps_vel + 1])
        self.lookup_table[0, 1:] = vels
        self.lookup_table[1:, 0] = steers

        for steer_idx, steer in enumerate(steers):
            for vel_idx, vel in enumerate(vels):
                initialState = [0, 0, 0, vel, 0, 0]
                u = [steer, 0]
                sol = self.sim.run_simulation(initialState, u)

                # check if sol[5] is does not change anymore - steady state reached
                if np.allclose(sol[-11:-1, 5], sol[-15:-5, 5], rtol=1e-3):
                    # record the final lateral acceleration
                    a_lat = sol[-1, 5] * vel
                    self.lookup_table[steer_idx + 1, vel_idx + 1] = a_lat
                else:
                    # No steady state solution found
                    # No need to continue with this steering angle for higher velocities
                    self.lookup_table[steer_idx + 1, vel_idx + 1:] = None
                    break

    # we don't want multiple steering angle and velocity combinations to have the same lateral acceleration
    # sol: only take the lower combinations of steering angle and velocity
    # in parallel find the upper limit for the achievable lateral acceleration
    def find_upper_limits(self):
        # find the first maximum for a velocity and then only keep values on the lower end, set everything above to nan
        for vel_idx in range(1, self.lookup_table.shape[1]):
            vel = self.lookup_table[0, vel_idx]
            a_lats = self.lookup_table[1:, vel_idx]
            a_lats = a_lats[~np.isnan(a_lats)]  # drop nans at the end
            d_a_lats = np.diff(a_lats)
            # find the global maximum
            max_idx = np.argmax(a_lats)
            if max_idx != 0:
                # check if there is a local maximum before max_idx
                d_a_lats = np.diff(a_lats[0:max_idx])
                local_max_idx = np.argwhere(d_a_lats < 0)
                if local_max_idx.size != 0:
                    max_idx = local_max_idx[0]

            # set everything above max_idx to nan
            self.lookup_table[max_idx + 1:, vel_idx] = None

    def plot_lookup(self):
        # Plot the lookup table as a surface3d with velocitz and steering angle on x and y axis
        fig = plt.figure()
        ax = fig.add_subplot(111, projection='3d')
        ax.set_xlabel(r'$v_x$ [m/s]')
        ax.set_ylabel(r'$\delta$ [rad]')
        # add some space between y label and y axis
        ax.yaxis.labelpad = 10
        ax.set_zlabel(r'$a_c$ [m/s$^2$]')
        ax.set_xlim(START_VEL, END_VEL)
        ax.set_ylim(START_STEER, END_STEER)
        ax.set_zlim(0, 10)

        X, Y = np.meshgrid(self.lookup_table[0, 1:], self.lookup_table[1:, 0])
        Z = self.lookup_table[1:, 1:]
        ax.plot_surface(X, Y, Z, cmap='viridis', edgecolor='none', alpha=0.6, zorder=0)

        azimuth = -160
        elevation = 15
        ax.view_init(elevation, azimuth)
        plt.tight_layout()

        # add an example lookup pair
        steer = 0.1
        vel = 3.0
        # find corresponding indices
        steer_idx = np.where(np.abs(self.lookup_table[1:, 0] - steer) < 0.01)[0]
        vel_idx = np.where(np.abs(self.lookup_table[0, 1:] - vel) < 0.1)[0]
        if len(steer_idx) == 0 or len(vel_idx) == 0:
            steer_idx = 100
            vel_idx = 100
        if len(steer_idx) > 1:
            steer_idx = steer_idx[0]
        if len(vel_idx) > 1:
            vel_idx = vel_idx[0]

        a_lat = self.lookup_table[steer_idx + 1, vel_idx + 1]
        ax.scatter(vel, steer, a_lat, c='r', marker='o', s=20, zorder=10)
        ax.plot([vel, vel], [steer, steer], [0, a_lat], c='r', linestyle='--', zorder=10)
        ax.plot([vel, vel], [START_STEER, steer], [0, 0], c='r', linestyle='--', zorder=10)
        ax.plot([START_VEL, vel], [steer, steer], [0, 0], c='r', linestyle='--', zorder=10)
        plt.show()

    def save_lookup(self):
        rospack = rospkg.RosPack()
        path = rospack.get_path('stack_master')
        timestamp = datetime.now().strftime("%m%d")
        filename = f"{timestamp}_LUT.csv"
        archive_folder = os.path.join(path, "config", self.racecar_version, "LUT", self.floor, "archive")
        if not os.path.exists(archive_folder):
            os.makedirs(archive_folder)
        file_path = os.path.join(archive_folder, filename)
        np.savetxt(file_path, self.lookup_table, delimiter=",")
        print(f"SAVED LOOKUP TABLE TO: {file_path}")

        if self.update_latest:
            latest_file = os.path.join(path, "config", self.racecar_version, "LUT", self.floor, "default.csv")
            # create a symlink to the latest model
            if os.path.exists(latest_file):
                os.remove(latest_file)
            os.symlink(file_path, latest_file)
            print(f"UPDATED LATEST LOOKUP TABLE TO POINT TO: {latest_file}")

def main(racecar_version: str, floor: str, update_latest: bool= True):
    generator: LookupGenerator = LookupGenerator(racecar_version, floor, update_latest)
    generator.run_generator()

if __name__ == "__main__":
    fire.Fire(main)
