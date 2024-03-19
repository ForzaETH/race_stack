import os
import numpy as np
import matplotlib.pyplot as plt
from numpy.linalg import lstsq
from scipy.ndimage import median_filter
from scipy.signal import sosfiltfilt, butter

from helpers.bagloader import load_bags
from helpers.load_model import get_dotdict
from helpers.save_model import save

g_ = 9.81

model_name_ = "NUC1_pacejka" #name + tire model name
model, tires = model_name_.split("_")
local_bag_dir = './data/' + model + "/motor" # path to folder with rosbag(s) with relevant experiment data

script_dir = os.path.dirname(os.path.realpath(__file__))
bag_dir_ = os.path.join(script_dir, local_bag_dir)

class RPMAnalyser:
  def __init__(self, model_name = model_name_, bag_dir = bag_dir_):
    print(f"Loading model {model_name}...")
    self.model = get_dotdict(model_name)

    # hangar
    # field_dict = {"/vesc/odom": ["twist.twist.linear.x", "twist.twist.linear.y", "twist.twist.angular.z"], 
    #               "/vesc/commands/motor/speed": ["data"],
    #               "/vesc/sensors/imu/raw": ["linear_acceleration.x", "linear_acceleration.y"]}
    self.cmd_topic = "/vesc/commands/motor/speed"
    field_dict = {"/ekf/odometry/filtered": ["twist.twist.linear.x"], 
                  self.cmd_topic: ["data"],
                  "/vesc/sensors/imu/raw": ["linear_acceleration.y"]}

    if (model == "SIM") :
      self.issim = True
      self.data = load_bags(bag_dir, field_dict)
    else :
      self.issim = False
      self.data = load_bags(bag_dir, field_dict)

    self.model.C_0v = 220

    self.transform_data()
    self.solve_rmp_fixed_offset()
    i = 1
    while (i < 4):
      self.discard_outliers(i)
      self.solve_rmp_fixed_offset()
      i += 1

    save(self.model)
    print("Final C_v: " + str(self.model.C_v))
    print("Final C_0v: " + str(self.model.C_0v))
    self.transform_data()
    self.plot_solution()
  
  def discard_outliers(self, i):
    print("Outlier Threshold: " + str(4000 * np.exp(-i)))
    error = abs(self.model.C_v * self.v_x + self.model.C_0v - self.erpm)
    keep_idxs = np.where(error <= 4000 * np.exp(-i))
    self.v_x = self.v_x[keep_idxs]
    self.erpm = self.erpm[keep_idxs]

  def transform_data(self):
    sos = butter(2, 0.05, output='sos')
    if (self.issim):
      self.a_long = self.data["linear_acceleration.x"].to_numpy()
    else:
      self.a_long_unfilt = - self.data["linear_acceleration.y"].to_numpy()
      self.a_long = sosfiltfilt(sos, self.a_long_unfilt)
    
    self.data = self.data[abs(self.a_long) < 0.2] 
    
    self.v_x = self.data["twist.twist.linear.x"].to_numpy()
    self.v_x = median_filter(self.v_x, size= 5)

    # self.v_x = sosfiltfilt(sos, self.v_x)
    self.erpm = self.data[self.cmd_topic+".data"].to_numpy()
    
  def solve_rmp(self):
    v_gain = np.reshape(self.v_x, (-1,1))
    A = np.concatenate((v_gain, np.ones_like(v_gain)), axis = 1)
    y = np.reshape(self.erpm, (-1,1))
    
    # front
    sol = lstsq(A, y)
    print("Found a least squares solution: ")
    print(sol)
    self.model.C_v = float(sol[0][0])
    self.model.C_0v = float(sol[0][1])

    # prepare plot:
    self.space = np.linspace(np.min(self.v_x), np.max(self.v_x), 50)
    self.fit = self.model.C_v * self.space + self.model.C_0v
  
  def solve_rmp_fixed_offset(self):
    v_gain = np.reshape(self.v_x, (-1,1))
    A = v_gain
    y = np.reshape(self.erpm, (-1,1))
    
    # front
    sol = lstsq(A, y)
    print("Found a least squares solution: ")
    print(sol)
    self.model.C_v = float(sol[0][0])

    # prepare plot:
    self.space = np.linspace(np.min(self.v_x), np.max(self.v_x), 50)
    self.fit = self.model.C_v * self.space + self.model.C_0v

  def plot_solution(self):
    plt.figure(figsize=(8,6))

    plt.scatter(self.v_x, self.erpm, color="Green", alpha=0.15, label='Measurements')
  
    plt.plot(self.space, self.fit, 'b', label=f'Model prediction')

    plt.xlabel(r'$v_x$ [m/s]')
    plt.ylabel(r'ERPM [1]')
    plt.grid()
    plt.legend(loc='best')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
  RPMAnalyser(model_name_, bag_dir_)
