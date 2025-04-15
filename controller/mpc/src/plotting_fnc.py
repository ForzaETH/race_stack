import matplotlib.pyplot as plt
import numpy as np

# TODO probably remove all these functionalities

def plot_res(spline, simX, simU, realX):
    # plot results
    t = np.linspace(0,len(simU),len(simU))
    figure = plt.figure()
    plt.subplot(2, 1, 1)
    plt.step(t, simU[:,0], color='r')
    plt.step(t, simU[:,1], color='g')
    plt.title("Result plots")
    plt.legend(['dD','ddelta'],bbox_to_anchor=(1.05, 1),loc='upper left', borderaxespad=0.)
    plt.ylabel('u')
    plt.xlabel('iteration')
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(t, simX[:,1:])
    plt.plot(t, realX[:,1:])
    plt.ylabel('x')
    plt.xlabel('iteration')
    plt.legend(['n','alpha','v','D','delta', 'real_n','real_alpha','real_v','real_D','real_delta'],bbox_to_anchor=(1.05, 1),loc='upper left', borderaxespad=0.)
    # figure.set_size_inches(1.8, 0.9)
    plt.grid(True)
    plt.show()
    figure = plt.figure()
    plt.plot(t, simX[:,0])
    plt.show()
    figure = plt.figure()
    sd = np.array(simX[:,:2])
    xy = convert_frenet_to_cartesian_plt(spline, sd)
    xy_desired = np.zeros((sd.shape))
    # xy_mintime = np.zeros((sd.shape))
    for i in range(sd.shape[0]):
        xy_desired[i,:] = spline.get_coordinate(sd[i,0])
        # xy_mintime[i,:] = mintime.get_coordinate(sd[i,0])
    plt.plot(-xy[:, 1], xy[:, 0])
    plt.plot(-xy_desired[:, 1], xy_desired[:, 0])
    # plt.plot(-xy_mintime[:, 1], xy_mintime[:, 0])
    plt.legend(["real path", "mincurve path"])
    plt.xlabel('$x$ [m]')
    plt.ylabel('$y$ [m]')
    plt.show()

def convert_frenet_to_cartesian_plt(spline, frenet_coords) -> np.array:

    co, si = np.cos(-np.pi/2), np.sin(-np.pi/2)
    R = np.array(((co, -si), (si, co)))

    if frenet_coords.shape[0] > 2:
        return np.array([spline.get_coordinate(s) + d*spline.get_derivative(s).reshape(1,2) @ R for s, d in frenet_coords]).reshape(-1,2)
    else:
        s = frenet_coords[0]
        d = frenet_coords[1]
        return spline.get_coordinate(s) + d*spline.get_derivative(s) @ R

def plot_costs(adv, n, alpha, reg):
    plt.plot(np.linspace(0,len(adv), len(adv)), adv)
    plt.plot(np.linspace(0,len(adv), len(adv)), n)
    plt.plot(np.linspace(0,len(adv), len(adv)), alpha)
    plt.plot(np.linspace(0,len(adv), len(adv)), reg)

    plt.legend(["advancement", "deviation", "alpha", "input"])
    plt.show()

def plot_n(simX, simU, realX):
    # plot results
    t = np.linspace(0,len(simU),len(simU))
    figure = plt.figure()
    plt.subplot(2, 1, 1)
    plt.step(t, simU[:,0], color='r')
    plt.step(t, simU[:,1], color='g')
    plt.title("Result plots")
    plt.legend(['dD','ddelta'],bbox_to_anchor=(1.05, 1),loc='upper left', borderaxespad=0.)
    plt.ylabel('u')
    plt.xlabel('iteration')
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(t, simX[:,1])
    plt.plot(t, realX[:,1])
    plt.ylabel('x')
    plt.xlabel('iteration')
    plt.legend([r'$n_{sim}$', r'$n_{meas}$'])
    # figure.set_size_inches(1.8, 0.9)
    plt.grid(True)
    plt.show()

def plot_alpha(simX, simU, realX):
    # plot results
    t = np.linspace(0,len(simU),len(simU))
    figure = plt.figure()
    plt.subplot(2, 1, 1)
    plt.step(t, simU[:,0], color='r')
    plt.step(t, simU[:,1], color='g')
    plt.title("Result plots")
    plt.legend(['dD','ddelta'],bbox_to_anchor=(1.05, 1),loc='upper left', borderaxespad=0.)
    plt.ylabel('u')
    plt.xlabel('iteration')
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(t, simX[:,2])
    plt.plot(t, realX[:,2])
    plt.ylabel('x')
    plt.xlabel('iteration')
    plt.legend([r'$\alpha_{sim}$', r'$\alpha_{meas}$'])
    # figure.set_size_inches(1.8, 0.9)
    plt.grid(True)
    plt.show()

def plot_vx(simX, simU, realX):
    # plot results
    t = np.linspace(0,len(simU),len(simU))
    figure = plt.figure()
    plt.subplot(2, 1, 1)
    plt.step(t, simU[:,0], color='r')
    plt.step(t, simU[:,1], color='g')
    plt.title("Result plots")
    plt.legend(['dD','ddelta'],bbox_to_anchor=(1.05, 1),loc='upper left', borderaxespad=0.)
    plt.ylabel('u')
    plt.xlabel('iteration')
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(t, simX[:,3])
    plt.plot(t, realX[:,3])
    plt.ylabel('x')
    plt.xlabel('iteration')
    plt.legend([r'$v_{x,sim}$', r'$v_{x,meas}$'])
    # figure.set_size_inches(1.8, 0.9)
    plt.grid(True)
    plt.show()

def plot_vy(simX, simU, realX):
    # plot results
    t = np.linspace(0,len(simU),len(simU))
    figure = plt.figure()
    plt.subplot(2, 1, 1)
    plt.step(t, simU[:,0], color='r')
    plt.step(t, simU[:,1], color='g')
    plt.title("Result plots")
    plt.legend(['dD','ddelta'],bbox_to_anchor=(1.05, 1),loc='upper left', borderaxespad=0.)
    plt.ylabel('u')
    plt.xlabel('iteration')
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(t, simX[:,4])
    plt.plot(t, realX[:,4])
    plt.ylabel('x')
    plt.xlabel('iteration')
    plt.legend([r'$v_{y,sim}$', r'$v_{y,meas}$'])
    # figure.set_size_inches(1.8, 0.9)
    plt.grid(True)
    plt.show()

def plot_r(simX, simU, realX):
    # plot results
    t = np.linspace(0,len(simU),len(simU))
    figure = plt.figure()
    plt.subplot(2, 1, 1)
    plt.step(t, simU[:,0], color='r')
    plt.step(t, simU[:,1], color='g')
    plt.title("Result plots")
    plt.legend(['dD','ddelta'],bbox_to_anchor=(1.05, 1),loc='upper left', borderaxespad=0.)
    plt.ylabel('u')
    plt.xlabel('iteration')
    plt.grid(True)
    plt.subplot(2, 1, 2)
    plt.plot(t, simX[:,5])
    plt.plot(t, realX[:,5])
    plt.ylabel('x')
    plt.xlabel('iteration')
    plt.legend([r'$r_{sim}$', r'$r_{meas}$'])
    # figure.set_size_inches(1.8, 0.9)
    plt.grid(True)
    plt.show()