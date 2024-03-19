import os
from scipy.optimize import least_squares
from scipy.ndimage import median_filter
import numpy as np
import pandas as pd
from numpy.linalg import lstsq
import matplotlib.pyplot as plt
from scipy.signal import sosfiltfilt, butter

from helpers.bagloader import load_bags
from helpers.load_model import get_dotdict
from helpers.save_model import save

g_ = 9.81

model_name_ = "NUC1_pacejka" #name + tire model name
model, tires = model_name_.split("_")
local_bag_dir = './data/' + model + "/tire_dynamics" # path to folder with rosbag(s) with relevant experiment data

script_dir = os.path.dirname(os.path.realpath(__file__))
bag_dir_ = os.path.join(script_dir, local_bag_dir)

class TireAnalyser:
  def __init__(self, model_name = model_name_, bag_dir = bag_dir_):
    print(f"Loading model {model_name}...")
    self.model = get_dotdict(model_name)

    # hangar
    # field_dict = {"/vesc/odom": ["twist.twist.linear.x", "twist.twist.linear.y", "twist.twist.angular.z"], 
    #               "/vesc/commands/motor/speed": ["data"],
    #               "/vesc/sensors/imu/raw": ["linear_acceleration.x", "linear_acceleration.y"]}
    self.cmd_topic = "/vesc/commands/servo/position"

    field_dict = {"/state_estimation/odom": ["twist.twist.angular.z"],
                  "/car_state/odom": ["twist.twist.linear.x", "twist.twist.linear.y"], 
                  #"/vesc/low_level/ackermann_cmd_mux/output": ["drive.speed", "drive.acceleration", "drive.steering_angle"],
                  self.cmd_topic: ["data"],
                  "/vesc/sensors/imu/raw": ["linear_acceleration.x", "linear_acceleration.y"]}

    sim_dict = {"/car_state/odom": ["twist.twist.linear.x", "twist.twist.linear.y", "twist.twist.linear.z", "twist.twist.angular.z"], 
                  "/nav": ["drive.speed", "drive.acceleration", "drive.steering_angle"],
                  "/imu": ["linear_acceleration.x", "linear_acceleration.y"]}

    if (model == "SIM") :
      self.issim = True
      self.data = load_bags(bag_dir, sim_dict)
    else :
      self.issim = False
      self.data = load_bags(bag_dir, field_dict)
    self.data = self.data[self.data["twist.twist.linear.x"] > 1.0] 

    self.analyse_tires()
    save(self.model)

  def analyse_tires(self):
    self.transform_data()
    self.discard_alpha_outliers()

    if self.model.tire_model == "linear":
      self.solve_linear_problem()
    if self.model.tire_model == "pacejka":
      self.solve_pacejka_problem()

    i = 1
    while i < 4: 
      self.discard_outliers(i)
      if self.model.tire_model == "linear":
        self.solve_linear_problem()
      if self.model.tire_model == "pacejka":
        self.solve_pacejka_problem()
      i += 1

    # reset outlier rejection for the final plot
    self.transform_data()
    self.plot_solution()

    # self.model.C_acc = sol[0][0]
    # self.model.C_R = sol[0][1]
  
  def transform_data(self):
    if (self.issim):
      self.a_long = self.data["linear_acceleration.x"].to_numpy()
      self.a_lat = self.data["linear_acceleration.y"].to_numpy()
    else :
      sos = butter(2, 0.05, output='sos')
      self.a_long_unfilt = - self.data["linear_acceleration.y"].to_numpy()
      self.a_long = sosfiltfilt(sos, self.a_long_unfilt)
      self.a_lat_unfilt = self.data["linear_acceleration.x"].to_numpy()
      self.a_lat = sosfiltfilt(sos, self.a_lat_unfilt)
    
    self.v_x = median_filter(self.data["twist.twist.linear.x"].to_numpy(), size=5)
    self.v_y = median_filter(self.data["twist.twist.linear.y"].to_numpy(), size=5)
    self.omega = self.data["twist.twist.angular.z"].to_numpy()
    
    self.delta = (self.data[self.cmd_topic + ".data"].to_numpy() - self.model.C_0d) / self.model.C_d
    
    self.F_zf = self.model.m *(g_ * self.model.l_r - self.a_long *self.model.h_cg) / (self.model.l_wb)
    self.F_zr = self.model.m *(g_ * self.model.l_f + self.a_long *self.model.h_cg) / (self.model.l_wb)
    self.meanFz_f = np.mean(self.F_zf)
    self.meanFz_r = np.mean(self.F_zr)
    self.alpha_f = -np.arctan((self.v_y + self.omega * self.model.l_f) / self.v_x) + self.delta
    self.alpha_r = -np.arctan(((self.v_y - self.omega * self.model.l_r) / self.v_x))

    self.F_yf = self.model.m * self.model.l_r * self.a_lat / ((self.model.l_r + self.model.l_f) * np.cos(self.delta))
    self.F_yr = self.model.m * self.model.l_f * self.a_lat / (self.model.l_r + self.model.l_f)

  def keep_only_at_indexes(self, keep_idxs):
    self.v_x = self.v_x[keep_idxs]
    self.v_y = self.v_y[keep_idxs]
    self.omega = self.omega[keep_idxs]
    self.delta = self.delta[keep_idxs]
    self.F_zf = self.F_zf[keep_idxs]
    self.F_zr = self.F_zr[keep_idxs]
    self.alpha_f = self.alpha_f[keep_idxs]
    self.alpha_r = self.alpha_r[keep_idxs]
    self.F_yf = self.F_yf[keep_idxs]
    self.F_yr = self.F_yr[keep_idxs]

  def discard_alpha_outliers(self):
    keep_idxs = np.where((abs(self.alpha_f) <= 2) & (abs(self.alpha_r) <= 2))
    self.keep_only_at_indexes(keep_idxs)

  def discard_outliers(self, i):
    print("threshold: " + str(20 * 2**(-i)))
    keep_idxs = np.where((self.error_f <= 20 * 2**(-i)) & (self.error_r <= 20 * 2**(-i)))
    self.keep_only_at_indexes(keep_idxs)

  def solve_linear_problem(self):
    A_f = np.reshape(self.model.mu*self.F_zf*self.alpha_f, (-1, 1))
    A_r = np.reshape(self.model.mu*self.F_zr*self.alpha_r, (-1, 1))

    y_f = np.reshape(self.F_yf, (-1,1))
    y_r = np.reshape(self.F_yr, (-1,1))
    
    # front
    sol_f = lstsq(A_f, y_f)
    print("Found a least squares solution for front tires: ")
    print(sol_f)
    self.model.C_Sf = float(sol_f[0][0])

    sol_r = lstsq(A_r, y_r)
    print("Found a least squares solution for rear tires: ")
    print(sol_r)
    self.model.C_Sr = float(sol_r[0][0])

    # prepare plot:
    self.space_f = np.linspace(-0.65, 0.65, 50)
    self.fit_f = self.model.mu * self.meanFz_f * self.model.C_Sf * self.space_f

    self.space_r = np.linspace(-0.65, 0.65, 50)
    self.fit_r = self.model.mu * self.meanFz_r * self.model.C_Sr * self.space_r

    self.error_f = abs(self.model.mu * self.F_zf * self.model.C_Sf * self.alpha_f \
                  - self.F_yf)
    self.error_r = abs(self.model.mu * self.F_zr * self.model.C_Sr * self.alpha_r \
                  - self.F_yr)

  def pacejka_formula(self, params, alpha, F_z):
    B, C, D, E = params
    y = self.model.mu * F_z * D * np.sin(C*np.arctan(B*alpha - E * (B*alpha -np.arctan(B * alpha))))
    return y

  def pacejka_error(self, params, *args):
    alpha, F_z, F_y = args 
    y = self.pacejka_formula(params, alpha, F_z)
    error = (y - F_y)**2
    # square error
    return error

  def solve_pacejka_problem(self):
    start_params = [1, 1, 1, 0.5]
    # front
    sol_f = least_squares(self.pacejka_error, start_params, args=(self.alpha_f, self.F_zf, self.F_yf), bounds=([0.0, 0.0, 0.0, 0.0], [25, 4.0, 2.0, 1.1]))
    print("Found a least squares solution for front tires: ")
    print(sol_f.x)
    self.model.C_Pf = sol_f.x.tolist()

    sol_r = least_squares(self.pacejka_error, start_params, args=(self.alpha_r, self.F_zr, self.F_yr), bounds=([0.0, 0.0, 0.0, 0.0], [20, 1.5, 2.0, 0.8]))
    print("Found a least squares solution for rear tires: ")
    print(sol_r.x)
    self.model.C_Pr = sol_r.x.tolist()

    # prepare plot:
    self.space_f = np.linspace(-0.65, 0.65, 50)
    self.fit_f = self.pacejka_formula(self.model.C_Pf, self.space_f, self.meanFz_f)

    self.space_r = np.linspace(-0.65, 0.65, 50)
    self.fit_r = self.pacejka_formula(self.model.C_Pr, self.space_r, self.meanFz_r)

    self.error_f = abs(self.pacejka_formula(self.model.C_Pf, self.alpha_f, self.F_zf) \
                  - self.F_yf)
    self.error_r = abs(self.pacejka_formula(self.model.C_Pr, self.alpha_r, self.F_zr) \
                  - self.F_yr)

  def plot_solution(self):
    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(8,6))
    # fig.suptitle('Tire Dynamics Identification')
    ax1.scatter(self.alpha_f, self.F_yf, color="Green", alpha=0.15, label='Est. Lateral Force')
    ax2.scatter(self.alpha_r, self.F_yr, color="Green", alpha=0.15, label='Est. Lateral Force')

    ax1.plot(self.space_f, self.fit_f, 'b', label=f'Model predicition for fixed Fz: {self.meanFz_f:.2f}')
    ax2.plot(self.space_r, self.fit_r[:], 'b', label=f'Model predicition for fixed Fz: {self.meanFz_r:.2f}')

    ax1.set_title('Front tires')
    ax2.set_title('Rear tires')
    for ax in [ax1, ax2]:
      ax.set_xlabel(r'$\alpha_f$ [rad]')
      ax.set_ylabel(r'$F_y$ [N]')
      ax.grid()
      ax.legend(loc='best')
      ax.set_xlim([-0.65, 0.65])
      ax.set_ylim([-20, 20])

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
  TireAnalyser(model_name_, bag_dir_)
  