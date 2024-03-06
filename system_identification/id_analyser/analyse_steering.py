import os
from scipy.ndimage import median_filter
import numpy as np
from numpy.linalg import lstsq
import matplotlib.pyplot as plt

from helpers.bagloader import load_bags
from helpers.load_model import get_dotdict
from helpers.save_model import save

g_ = 9.81

model_name_ = "NUC1_pacejka" #name + tire model name
model, tires = model_name_.split("_")
local_bag_dir = './data/' + model + "/steering_angle" # path to folder with rosbag(s) with relevant experiment data

script_dir = os.path.dirname(os.path.realpath(__file__))
bag_dir_ = os.path.join(script_dir, local_bag_dir)

class SteeringAnalyser:
  def __init__(self, model_name = model_name_, bag_dir = bag_dir_):
    print(f"Loading model {model_name}...")
    self.model = get_dotdict(model_name)

    # simulation
    # field_dict = {"/odom": ["twist.twist.linear.x", "twist.twist.linear.y", "twist.twist.angular.z"], 
    #               "/drive": ["drive.steering_angle"]}

    # hangar
    self.cmd_topic = "/vesc/commands/servo/position"
    field_dict = {"/car_state/odom": ["twist.twist.linear.x", "twist.twist.linear.y", 
                               "twist.twist.angular.z"],
                  self.cmd_topic: ["data"]}

    self.data = load_bags(bag_dir, field_dict)
    self.transform_data()

    i = 0
    while (i < 9):
      self.solve_steering()
      self.discard_outliers(i)
      i += 1 

    save(self.model)
    print("Final C_d: " + str(self.model.C_d))
    print("Final C_0d: " + str(self.model.C_0d))
    self.transform_data()
    self.plot_solution()

  def transform_data(self):
    self.omega = self.data["twist.twist.angular.z"].to_numpy()
    self.omega = median_filter(self.omega, size=5)
    self.v_x = self.data["twist.twist.linear.x"].to_numpy()
    self.v_x = median_filter(self.v_x, size=5)
    self.servo_position = self.data[self.cmd_topic + ".data"].to_numpy()

  def solve_steering(self):
    self.atan = np.arctan2((self.model.l_r + self.model.l_f) * self.omega, self.v_x)
    atan_vector = np.reshape(self.atan, (-1, 1))
    A_i = np.concatenate((atan_vector, np.ones_like(atan_vector)), axis=1)
    y_i = np.reshape(self.servo_position, (-1, 1))
    sol = lstsq(A_i, y_i)
    print("Found a least squares solution: ")
    print(sol)
    self.model.C_d = float(sol[0][0])
    self.model.C_0d = float(sol[0][1])
    self.fit = (self.servo_position - self.model.C_0d * \
        np.ones_like(self.servo_position)) / self.model.C_d

  def discard_outliers(self, i):
    print("Outlier Threshold: " + str(4 * 2**(-i)))
    error = abs(self.fit - self.atan)
    keep_idxs = np.where(error <= 4 * 2**(-i))
    self.v_x = self.v_x[keep_idxs]
    self.omega = self.omega[keep_idxs]
    self.servo_position = self.servo_position[keep_idxs]

  def plot_solution(self):
    self.atan = np.arctan2((self.model.l_r + self.model.l_f) * self.omega, self.v_x)

    self.fit = (self.servo_position - self.model.C_0d * \
        np.ones_like(self.servo_position)) / self.model.C_d

    fig, axs = plt.subplots(1, figsize=(8,5))
    plt.xlabel(r'Normalized Time')

    cmd_axs = axs.twinx()
    p3, = cmd_axs.plot(np.linspace(0, 1,num=len(self.servo_position)), self.servo_position, 'r', label='com. servo position')

    p1, = axs.plot(np.linspace(0, 1,num=len(self.atan)), self.atan, 'g', alpha=0.7, label='est. measured steering angle')
    p2, = axs.plot(np.linspace(0, 1,num=len(self.fit)), self.fit, 'b', label='predicted steering angle')
    axs.set_ylabel(r'Steering Angle [$rad$]')
    axs.set_ylim(-1.0, 1.0)
    axs.grid()

    cmd_axs.set_ylabel(r'Servo Position [0-1]')
    cmd_axs.yaxis.label.set_color("Red")
    cmd_axs.spines["right"].set_edgecolor("Red")
    cmd_axs.tick_params(axis='y', colors="Red")
    cmd_axs.set_ylim(0.0,1.0)

    lines = [p1, p2, p3]
    axs.legend(lines, [l.get_label() for l in lines])

    plt.xlim(0.0, 1.0)
    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
  Analyser = SteeringAnalyser(model_name_, bag_dir_)
