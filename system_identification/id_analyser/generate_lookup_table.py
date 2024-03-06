import os
from scipy.integrate import odeint
import numpy as np
import matplotlib.animation as animation
import matplotlib.pyplot as plt

from helpers.vehicle_dynamics import vehicle_dynamics
from helpers.load_model import get_dotdict

ANIMATE = False
PLOT = False

model_name = "NUC1_pacejka"
model, tiretype = model_name.split("_")

model = get_dotdict(model_name)

START_STEER = 0.0
STEER_FINE_END = 0.1
END_STEER = 0.4
N_STEPS_STEER = 30 # used for fine region and then again for coarse region
START_VEL = 0.5
END_VEL = 7.0
N_STEPS_VEL = 65

class Simulator:
  def func_ST(self, x, t, u, p):
      f = vehicle_dynamics(x, u, p, tiretype)
      return f

  def animate(self, i):
    self.loc.set_data(self.sol[i, 0], self.sol[i, 1])
    self.loc.set_3d_properties(0)
    return self.loc

  def generate_lookup(self):
    fine_steers = np.linspace(START_STEER, STEER_FINE_END, N_STEPS_STEER)
    steers = np.linspace(STEER_FINE_END, END_STEER, N_STEPS_STEER)
    steers = np.concatenate((fine_steers, steers))
    vels = np.linspace(START_VEL, END_VEL, N_STEPS_VEL)

    total_steps_steer = len(steers)
    self.lookup_table = np.empty([total_steps_steer + 1, N_STEPS_VEL +1])    

    self.lookup_table[0, 1:] = vels
    self.lookup_table[1:, 0] = steers

    for steer_idx, steer in enumerate(steers):
      for vel_idx, vel in enumerate(vels):
        initialState = [0, 0, 0, vel, 0, 0]
        u = [steer, 0]
        
        # print("Steering Angle :" + str(steer) + " Velocity :" + str(vel))
        dt = 0.01
        duration = 2.0
        t = np.arange(0, duration, dt)
        frames = np.arange(0, len(t), 1)

        self.sol = odeint(self.func_ST, initialState, t, args=(u, model))  

        if abs(self.sol[-1, 5] - self.sol[-10, 5]) > 0.05:
          # print("failed to converge, limit")
          self.lookup_table[steer_idx+1, vel_idx+1:] = None
          break
        else:
          # save the final lateral acceleration
          a_lat = self.sol[-1, 5] * vel
          # a_lat_all = self.sol[:, 5] * vel
          self.lookup_table[steer_idx+1, vel_idx+1] = a_lat 

        if PLOT:
          fig = plt.figure(figsize=(12, 12))
          ax1 = fig.add_subplot(2, 2, 1)
          ax1.plot(t, self.sol[:, 4], 'b', label='vy(t)')
          plt.legend(loc='best')
          plt.xlabel('t')
          plt.grid()

          ax2 = fig.add_subplot(2, 2, 3)
          ax2.plot(t, self.sol[:, 5], 'b', label='yaw rate(t)')
          plt.legend(loc='best')
          plt.xlabel('t')
          plt.grid()

          ax4 = fig.add_subplot(2, 2, 4)
          ax4.plot(t, self.sol[:, 2], 'b', label='yaw(t)')
          plt.legend(loc='best')
          plt.xlabel('t')
          plt.grid()

          ax3 = fig.add_subplot(2, 2, 2)
          ax3.plot(self.sol[:, 0], self.sol[:, 1], 'r', label='position')
          plt.legend(loc='best')
          plt.xlabel('t')
          plt.grid()
          
          plt.show()

        if ANIMATE:
          fig = plt.figure(figsize=(10, 10))
          self.ax = fig.add_subplot(111, projection='3d', autoscale_on=True)
          self.loc, = self.ax.plot([],[],[], 'o-', color="blue")
          self.ax.set_xlim(self.sol[0, 0], self.sol[-1, 0])
          self.ax.set_ylim(self.sol[0, 1], self.sol[-1, 1])
          self.ax.set_zlim(0, 1)
          self.ax.margins(1, tight=False)
          self.ax.autoscale_view()
          self.ax.grid()
          ani = animation.FuncAnimation(fig, self.animate, frames=frames, interval=1, blit = False, repeat=False)

          ani.running = True
          plt.show()
          ani.running = False

  def save_lookup(self):
    model, tires = model_name.split("_")
    script_dir = os.path.dirname(__file__)
    lookup_dir = os.path.join(script_dir, "models", model)
    os.makedirs(lookup_dir, exist_ok=True)
    file_path = os.path.join(lookup_dir, model_name + "_lookup_table.csv")
    np.savetxt(file_path, self.lookup_table, delimiter=",")

if __name__ == "__main__":
  sim = Simulator()
  sim.generate_lookup()
  sim.save_lookup()
  print("Done, don't forget to copy to steering_lookup/cfg")