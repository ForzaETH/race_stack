import os
import numpy as np
import pandas as pd
from numpy.linalg import lstsq
import matplotlib.pyplot as plt

from helpers.bagloader import load_bags
from helpers.load_model import get_dotdict
from helpers.save_model import save

model_name_ = "SIM1_linear" #name + tire model name
local_bag_dir = 'data/SIM1/accel' # path to folder with rosbag(s) with relevant experiment data

script_dir = os.path.dirname(os.path.realpath(__file__))
bag_dir_ = os.path.join(script_dir, local_bag_dir)

# needs to have acceleration analysed before as C_R is fixed for this

class DecelerationAnalyser:
  def __init__(self, model_name = model_name_, bag_dir = bag_dir_):
    print(f"Loading model {model_name}...")
    self.model = get_dotdict(model_name)

    # hangar
    self.cmd_topic = "/vesc/commands/motor/brake"
    field_dict = {"/vesc/sensors/imu/raw": [ "linear_acceleration.y"], 
                  "/vesc/odom": [ "twist.twist.linear.x"], 
                  self.cmd_topic: ["data"]}
    self.data = load_bags(bag_dir, field_dict)

    self.analyse_decel()
    self.refine_solution()
    self.plot_solution()

    save(self.model)

  def analyse_decel(self):
    sol = self.solve_ls_problem()
    self.model.C_dec = sol[0][0]

  def refine_solution(self):
    # refine solution, discard any datapoints that lie outside of std of the fit
    # calculate the fit error for this data 
    error = (self.model.C_dec * self.accel + self.model.C_R * self.velocity) - self.current
    error_std = np.std(error)

    error_df = pd.DataFrame(error, columns=["error"])
    self.data = pd.concat([self.data, error_df], axis=1)
    self.data = self.data[self.data.error <= error_std] 
    sol = self.solve_ls_problem()
    self.model.C_dec = float(sol[0][0])

  def solve_ls_problem(self):
    self.current = - self.data[self.cmd_topic + ".data"].to_numpy()
    self.velocity = self.data["twist.twist.linear.x"].to_numpy()
    
    # time = data["Time"]
    self.accel = -self.data["linear_acceleration.y"].to_numpy()

    v = np.reshape(self.velocity, (-1, 1)) # column vector
    a = np.reshape(self.accel, (-1, 1))  # column vector
    y_i = np.reshape(self.current, (-1, 1)) - self.model.C_R * v# column vector
    A_i = a
    sol = lstsq(A_i, y_i)

    print("Found a least squares solution: ")
    print(sol)
    return sol

  def plot_solution(self):
    fit = np.divide((self.current - self.model.C_R * self.velocity), self.model.C_dec) # predicted acceleration

    fig, axs = plt.subplots(1, figsize=(7,5))

    length = len(self.accel)
    axs.plot(np.linspace(0,1,length), self.accel[:], 'b', label='est. x acceleration')
    axs.plot(np.linspace(0,1,length), self.current[:]*0.05, 'r-', alpha=.5, label='com. current / 100')
    axs.plot(np.linspace(0,1,length), self.velocity[:], 'r:', alpha=.5, label='est. velocity')
    axs.plot(np.linspace(0,1,length), fit[:], 'g', label='predicted acceleration')


    for ax in [axs]:
        ax.set_xlabel('Normalized Time')
        ax.set_ylabel(r'Acceleration [$\frac{m}{s^2}$]')
        ax.grid()
        ax.legend(loc='best')

    plt.tight_layout()
    plt.show()

if __name__ == '__main__':
  DecelerationAnalyser(model_name_, bag_dir_)
