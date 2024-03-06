from helpers.save_model import save
# from helpers.dotdict import DotDict #only used in the experiments, not for pickle dump

## Model Parameters that need to be measured manually ##
model_name = "SIM1"      # name
tire_model = 'linear'   # currently suppoted: linear, pacejka (SIM only models linear tire slip)
m = 3.47                # mass in kg # NUC1 3.54, SIM 3.47 
l_wb = 0.3302           # length of the wheelbase in m (front to back) # NUC1 0.307, SIM 0.3302
l_f = 0.15875           # distance of CoM to front axle in m # NUC1 0.162, SIM 0.15875
l_r = l_wb - l_f        # distance of CoM to rear axle in m 
h_cg = 0.074            # height of the center of mass above the wheel axis in m # NUC1 0.014, SIM 0.074
I_z = 0.04712           # yawing moment of inertia in kg*m^2 # NUC1 0.05797, SIM 0.04712

# default parameters that the model is initialized with, overwritten in the experiments
mu = 0.523              # friction coefficient, fixed for first set of experiments # RW 1, SIM 0.523 
C_d = 1.1               # gain from servo position to steering angle
C_0d = 0.02             # offset from servo position to steering angle
C_v = 3500.0            # gain from velocity to erpm 
C_0v = 0.0              # offset from velocity to erpm
C_acc = 3.2             # gain from acceleration command to current
C_dec = 3.2             # gain from deceleration command to current
C_R = 0.1               # gain from velocity (and hence induced drag and friction) to current
a_min = -3              # minimum acceleration in m*s^-2
a_max = 3               # maximum acceleration in m*s^-2
C_Pf = [0.1, 0.1, 0.1, 0.1] # pacejka tire parameters front
C_Pr = [0.1, 0.1, 0.1, 0.1] # pacejka tire parameters rear
C_Sf = 4.718              # cornering stiffness front axle 
C_Sr = 5.4562              # cornering stiffness rear axle

#Other Setup Parameters
overwrite_existing = True  #only set to true if you are sure what you are doing

def add_model():
  #Do work:
  model_dict = {
    'model_name': model_name,
    'tire_model': tire_model,
    'm': m,
    'l_wb': l_wb,
    'l_f': l_f,
    'l_r': l_r,
    'h_cg': h_cg,
    'I_z': I_z,
    'mu': mu,
    'C_d': C_d,
    'C_0d': C_0d,
    'C_v': C_v,
    'C_0v': C_0v,
    'C_acc': C_acc,
    'C_dec': C_dec,
    'a_min': a_min,
    'a_max': a_max,
  }
  if tire_model == 'linear':
    model_dict['C_Sf'] = C_Sf
    model_dict['C_Sr'] = C_Sr
  if tire_model == 'pacejka':
    model_dict['C_Pf'] = C_Pf
    model_dict['C_Pr'] = C_Pr

  save(model_dict, overwrite_existing, verbose=True)

if __name__ == '__main__':
    add_model()
