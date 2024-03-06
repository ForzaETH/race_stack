#!/usr/bin/env python3
import rospy
import rospkg
import actionlib
from bayesopt4ros.msg import BayesOptAction, BayesOptGoal
from dynamic_reconfigure.msg import Config
import dynamic_reconfigure.client
from f110_msgs.msg import LapData
from std_msgs.msg import Float32
import numpy as np
from bayesopt4ros import test_objectives
import matplotlib.pyplot as plt
import os
import yaml
import sys
import subprocess

# testing git stuff


class BayesOptimizer:
    def __init__(self, name):
        """
        This node optimizes some parameters of the MAP controller (L1 distance parameter) and also the sector scalers.
        """
        self.name = name

        ##########################################################
        # for visualization
        save_plot_bool = True
        save_location = "Downloads"
        ###########################################################

        self.car_distance_to_boundary = []
        self.opt_l1params = rospy.get_param("param_optimizer/opt_l1params")
        self.opt_sector_scalers = rospy.get_param(
            "param_optimizer/opt_sector_scalers")
        self.iterations = rospy.get_param("param_optimizer/iterations")
        self.laps_per_experiment = rospy.get_param(
            "param_optimizer/laps_per_experiment"
        )
        # cost scalers:
        self.cost_lap_time_scaler = rospy.get_param(
            "param_optimizer/cost_lap_time_scaler"
        )
        self.cost_crash_avoidance_scaler = rospy.get_param(
            "param_optimizer/cost_crash_avoidance_scaler"
        )
        self.cost_lat_err_scaler = rospy.get_param(
            "param_optimizer/cost_lat_err_scaler"
        )

        # shut down all nodes if both tune variables are False
        if not (self.opt_l1params) and not (self.opt_sector_scalers):
            rospy.logwarn(
                f"[{self.name}] Change self.opt_l1params and/or self.opt_sector_scalers to True"
            )
            subprocess.call(["rosnode", "kill", "BayesOptNode"])
            sys.exit()

        self.fastest_lap_time = None
        rospy.Subscriber("lap_data", LapData, self.lap_data_cb)  # lap data
        rospy.Subscriber(
            "estimated_lap_time", Float32, self.fastest_lap_time_cb
        )  # est lap time is actually fastest lap time # TODO change name
        rospy.Subscriber(
            "/min_car_distance_to_boundary", Float32, self.get_distance_to_boundary_cb
        )  # min distance to boundary

        self.sim = rospy.get_param("/sim")

        # Init at zero:
        self.n_sectors = 0
        self.number_of_l1_param = 0

        # initialize dynamic reconfig client
        if self.opt_l1params:
            rospy.Subscriber(
                "/l1_param_tuner/parameter_updates", Config, self.l1_params_cb
            )  # l1 param updating
            self.number_of_l1_param = (
                3  # we consider 3 variables for l1 parameter (see README)
            )
            self.dyn_rec_client_l1params = dynamic_reconfigure.client.Client(
                "l1_param_tuner", timeout=30
            )

        if self.opt_sector_scalers:
            # get initial sector scaling
            self.sectors_params = rospy.get_param("/map_params")
            self.n_sectors = self.sectors_params["n_sectors"]
            self.dyn_rec_client_sectors = dynamic_reconfigure.client.Client(
                "dyn_sector_server", timeout=30
            )
            self.set_sector_global_limit()
            # Write map_BO_forrester.yaml file

        # update Bayes Optimization .yaml-file: Edit number of parameters and ranges
        self.update_yaml_file()

        # Bayesian Opt client:
        self.bayes_opt_client = actionlib.SimpleActionClient(
            "BayesOpt", BayesOptAction)
        try:
            self.bayes_opt_client.wait_for_server(
                timeout=rospy.Duration.from_sec(1.0)
            )  # wait for bayesOpt server
        except rospy.exceptions.ROSException:
            rospy.logwarn(f"[{self.name}] BayesOpt server is not responsding")

        # for plotting/visualizing
        self.lap_time_list = []
        self.avg_lat_err_list = []
        self.min_car_dist_list = []
        self.y_list = []
        self.param_list = []
        self.y_opt_list = []
        self.skipped_iterations = []

        # For smallest measured values:
        self.shortest_lap_time = 999.9
        self.smallest_avg_lat_err = 9.9
        # For lap_time and error corresponding to the found optimum
        self.optimal_lap_time = 999.9
        self.optimal_avg_lat_err = 9.9

        # call optimize_loop
        self.optimize_loop()

        self.visualize_data(save_plot_bool, save_location)

    #############
    # CALLBACKS #
    #############
    def get_distance_to_boundary_cb(self, data):
        self.min_car_distance = data.data

    def fastest_lap_time_cb(self, data):
        self.fastest_lap_time = (
            data.data
        )  # estimated_lap_time topic is actually the fastest lap time...

    def lap_data_cb(self, msg):
        """
        extract lap data from lap_data topic
        """
        self.avg_lat_err = msg.average_lateral_error_to_global_waypoints
        self.lap_time = msg.lap_time
        self.lap_count = msg.lap_count

    def l1_params_cb(self, params: Config):
        """
        Here the l1 parameters will be updated accordingly
        """
        self.t_clip_min = params.doubles[0].value
        self.m_l1 = params.doubles[2].value
        self.q_l1 = params.doubles[3].value
        self.l1_default_config = [self.t_clip_min, self.m_l1, self.q_l1]

    #########
    # UTILS #
    #########
    def update_yaml_file(self):
        """
        This function retrieves the yaml data and transfers the ranges from BO_ranges.yaml to map_BO_forrester.yaml:
            The input dimension and parameters ranges are edited.
            This step is necessary, as the number of sectors varies.
        """
        number_of_sectors = self.n_sectors
        self.lower_bound = []
        self.upper_bound = []

        BO_ragnes_path = rospkg.RosPack().get_path("stack_master")
        # Load yaml file where you put in ranges for each parameter/sectors
        with open(
            BO_ragnes_path
            + "/config/"
            + self.get_racecar_version()
            + "/BO_ranges.yaml",
            "r",
        ) as file:
            BO_ranges_data = yaml.safe_load(file)

        # update bounds:
        if self.opt_l1params:
            t_clip_min_lb = BO_ranges_data["l1_param"]["t_clip_min"]["lower_bound"]
            t_clip_min_ub = BO_ranges_data["l1_param"]["t_clip_min"]["upper_bound"]
            m_l1_lb = BO_ranges_data["l1_param"]["m_l1"]["lower_bound"]
            m_l1_ub = BO_ranges_data["l1_param"]["m_l1"]["upper_bound"]
            q_l1_lb = BO_ranges_data["l1_param"]["q_l1"]["lower_bound"]
            q_l1_ub = BO_ranges_data["l1_param"]["q_l1"]["upper_bound"]

            self.lower_bound = [t_clip_min_lb, m_l1_lb, q_l1_lb]
            self.upper_bound = [t_clip_min_ub, m_l1_ub, q_l1_ub]

        if self.opt_sector_scalers:
            for idx in range(number_of_sectors):
                key_str = "Sector" + str(idx)  # e.g. 'Sector0'

                self.lower_bound.append(BO_ranges_data[key_str]["lower_bound"])
                self.upper_bound.append(BO_ranges_data[key_str]["upper_bound"])

        # Load the YAML file (map_BO_forrester.yaml):
        yaml_path = rospy.get_param("/bayesopt_config")
        with open(yaml_path, "r") as file:
            yaml_data = yaml.safe_load(file)

        # Update
        yaml_data["input_dim"] = (
            self.number_of_l1_param + number_of_sectors
        )  # edit number of parameters to optimize
        yaml_data["lower_bound"] = self.lower_bound
        yaml_data["upper_bound"] = self.upper_bound
        yaml_data["max_iter"] = self.iterations
        yaml_data["log_dir"] = (
            rospkg.RosPack().get_path("param_optimizer") + "/logs"
        )  # save logs in param_optimizer/logs

        # Save the updated YAML file
        with open(yaml_path, "w") as yaml_file:
            yaml.dump(yaml_data, yaml_file, default_flow_style=False)
        rospy.loginfo(f"[{self.name}] New values were stored in {yaml_path}")
        rospy.loginfo(f"[{self.name}] YAML UPDATED")

    def calculate_cost(self):
        """
        Calculate cost which has to be minimized. We want to minimize lap time, lateral error and
        maximize min distance to track boundary (to avoid crashes).
        ----------
        Output:
            cost: a function of lap time, average lateral error and min distance to track boundary
        """
        cost_lap_time = self.get_lap_time_cost()
        cost_crash_avoidance = self.get_distance_to_boundary_cost()
        cost_lat_err = self.get_avg_lat_err_cost()
        rospy.loginfo(f"[{self.name}] ----------- Calculate Cost -----------")
        rospy.loginfo(f"[{self.name}] Min distance: {self.min_car_distance}")
        rospy.loginfo(f"[{self.name}] Min distance cost: {cost_crash_avoidance}")
        rospy.loginfo(f"[{self.name}] Avg lat error: {self.avg_lat_err}")
        rospy.loginfo(f"[{self.name}] Avg lat error cost: {cost_lat_err}")
        rospy.loginfo(f"[{self.name}] Lap time: {self.lap_time}")
        rospy.loginfo(f"[{self.name}] Lap time cost: {cost_lap_time}")

        cost_total = (
            self.cost_lap_time_scaler * cost_lap_time
            + self.cost_crash_avoidance_scaler * cost_crash_avoidance
            + self.cost_lat_err_scaler * cost_lat_err
        )

        cost_total = cost_total / (
            self.cost_lat_err_scaler
            + self.cost_crash_avoidance_scaler
            + self.cost_lap_time_scaler
        )  # normalize cost
        rospy.loginfo(f"[{self.name}] = Cost total: {cost_total}")
        return cost_total

    def get_avg_lat_err_cost(self):
        standard_error = 0.15  # where the cost is 0.5
        slope = 50
        lateral_error_cost = 1 / (
            1 + np.exp(-slope * (self.avg_lat_err - standard_error))
        )
        return lateral_error_cost

    def get_distance_to_boundary_cost(self):
        """
        This function calculates a cost with the minimum boundary distance as an argument.
        If the car drives somwhere in the middle of the track far away from the boundary the cost is 0,
        as such a behavior is not problematic. At a certain distance to the track boundary (safety margin)
        the cost increases quadratic until it reaches a certain critical distance.

        Critical distance is where the car would touch the boundary and distances shorter than that is basically
        a "crash" (cost is 1)
        ----------
        OUTPUT:
            value: a value between 0 (best case) and 1 (worst case)
        """
        car_width = 0.28  # car width (28cm)
        critical_dist = (
            car_width / 2
        )  # distance where the car is touching boundary (half of the car width)
        safety_margin = (
            0.2 + car_width / 2
        )  # general safety margin to track boundary (20cm) from car wheels (0.014)

        # car distance cost function
        if self.min_car_distance < critical_dist:
            return 1
        elif self.min_car_distance > safety_margin:
            return 0
        else:
            return (
                1
                / ((critical_dist - safety_margin) ** 2)
                * (self.min_car_distance - safety_margin) ** 2
            )

    def get_lap_time_cost(self):
        """
        This function returns a value between 0 (optimal case) and 1 (worst case)
        The fastest lap time is the shortest lap time that a car can physically do (cost = 0)
        In the algorithm below there is a timeout at 2*fastest_lap_time which is why
        the cost is maximal 1 (worst case)
        """
        cost = self.lap_time / self.fastest_lap_time - 1
        return cost

    def update_parameters(self, param_new):
        """
        This function updates the parameters of interest with their new value.
        ----------
        Input:
            param_new: New values
        """
        control_length = len(param_new[: self.number_of_l1_param])
        control_length += len(param_new[self.number_of_l1_param:])
        if control_length != (self.n_sectors + self.number_of_l1_param):
            # for bug handling
            rospy.logerr(f"[{self.name}] rerun the script with SAME settings !!!")
            rospy.logerr(f"[{self.name}] parameter length do not match")
            subprocess.call(["rosnode", "kill", "BayesOptNode"])
            sys.exit()

        if self.opt_l1params:
            self.update_l1_param(
                param_new[: self.number_of_l1_param]
            )  # first few variables are l1 params values
        if self.opt_sector_scalers:
            self.update_sector_scalers(
                param_new[self.number_of_l1_param:]
            )  # the rest are sector scalers

    def update_l1_param(self, param_new):
        """
        Update new l1 parameters using dynamic reconfigure
        ----------
        Input:
            param_new: New l1 parameters
        """
        config = {
            "t_clip_min": float(param_new[0]),
            "m_l1": float(param_new[1]),
            "q_l1": float(param_new[2]),
        }
        self.dyn_rec_client_l1params.update_configuration(config)

    def update_sector_scalers(self, param_new):
        """
        Update new sector scalers using dynamic reconfigure
        ----------
        Input:
            param_new: New sector scalers
        """
        config = {}
        for i in range(self.n_sectors):
            key_name = "Sector" + str(i)
            config[key_name] = param_new[i]
        self.dyn_rec_client_sectors.update_configuration(config)

    def set_sector_global_limit(self):
        rospy.loginfo(f"[{self.name}] Setting global sector limit to 1.0")
        if self.sim:
            config = {
                "global_limit": 2.0,
            }
        else:
            config = {
                "global_limit": 1.0,
            }
        self.dyn_rec_client_sectors.update_configuration(config)

    def testfunction(self, Xin):
        """
        Test function (only to test bayesOpt package)
        optimum = minimum = 5
        optimal Xin = [0.33, 0.33, ...]
        """
        total = 5.0
        total += np.sum(np.power(Xin - 0.33, 2))
        return total

    def request_parameter(self, y_new: float) -> np.ndarray:
        """
        Method that requests new parameters from the BayesOpt server.
        Parameters
        ----------
        INPUT :
            y_new: The function value obtained from the objective/experiment.
        OUTPUT :
            result.x_new: the new values from BayesOpt server
        -------
        numpy.ndarray
            An array containing the new parameters suggested by BayesOpt server.
            Order of L1 params: t_clip_min, m_l1, q_l1
        """
        goal = BayesOptGoal(y_new=y_new)
        self.bayes_opt_client.send_goal(goal)
        self.bayes_opt_client.wait_for_result()
        result = self.bayes_opt_client.get_result()
        return result.x_new  # its called x_new in the bayesopt package

    #############
    # MAIN LOOP #
    #############
    def optimize_loop(self):
        """
        Each iteration is done with updated/new values for the parameter of interest (l1 and sector scalers).
        The set of new values is given by the bayesian optimization server.
        After some laps (minimum 1) the cost is calculated and sent to the same server to reveice the next new set of values.
        If the optimization is done, the optimal values are set with dynamic reconfigure.
        ----------
        Input:
            NONE
        Output:
            NONE
        """
        scaler = 2.0  # for timeout and punishment

        try:
            rospy.wait_for_message("estimated_lap_time", LapData, timeout=30)
        except rospy.exceptions.ROSException:
            rospy.WARN("Timed out waiting for message on 'estimated_lap_time'")

        # initialize variables with some values
        init_lap_time = (
            self.fastest_lap_time * scaler
        )  # also used as upper bound, later used for timeout also
        self.lap_time = init_lap_time
        init_avg_lat_err = 0.3  # high value
        self.avg_lat_err = init_avg_lat_err

        # Timeout and upper bounds for laptime
        self.experiment_timeout = init_lap_time  # [s]
        self.min_car_distance = 0  # for punishment calculation
        # punishment cost if lap time takes too long (longer than experiment_timeout)
        high_cost_punishment = 1
        y_opt = high_cost_punishment
        self.y_opt_list.append(y_opt)
        y_new = high_cost_punishment  # initialize with a high value
        self.y_list.append(y_new)

        # print settings in terminal for overview
        rospy.loginfo(f"[{self.name}] ####################### S E T T I N G S #######################")
        if self.opt_l1params:
            rospy.loginfo(f"[{self.name}] # Optimizing L1 parameter")
        if self.opt_sector_scalers:
            rospy.loginfo(f"[{self.name}] # Optimizing sector scaler")
        rospy.loginfo(f"[{self.name}] # Iterations: {self.iterations} with {self.laps_per_experiment} laps per experiment"
        )
        rospy.loginfo(f"[{self.name}] Ranges: ")
        rospy.loginfo(f"[{self.name}] lower bound:[{self.lower_bound}]")
        rospy.loginfo(f"[{self.name}] upper bound:[{self.upper_bound}]")
        rospy.loginfo(f"[{self.name}] ###############################################################")

        # wait for lap data = wait for current lap to be done
        try:
            rospy.wait_for_message("lap_data", LapData, timeout=60)
        except rospy.exceptions.ROSException:
            rospy.WARN("Timed out waiting for message on 'lap_data'")

        # start iteration
        for ite in range(1, 1 + self.iterations):
            param_new = self.request_parameter(y_new)
            self.param_list.append(param_new)
            self.update_parameters(param_new)

            # start experiment to calculate new y
            # temporary y only to calculate an average if lap per experiment is greater than 1
            y_temporary = 0

            self.car_distance_to_boundary = []  # reset array:

            # For plot
            lap_time_avg = 0
            avg_lat_err_avg = 0

            param_reasonable = True
            for lap_count in range(1, self.laps_per_experiment + 1):
                # wait for new lap data = wait until lap is done
                try:
                    rospy.wait_for_message(
                        "lap_data", LapData, timeout=self.experiment_timeout
                    )  # timeout such that we do not loose time with unsreasonable params
                except rospy.exceptions.ROSException:
                    rospy.logwarn(f"[{self.name}] Lap takes too long, not reasonable parameters. Skip! Press R1"
                    )
                    param_reasonable = False
                    self.skipped_iterations.append(ite)
                    if self.opt_l1params:
                        self.update_parameters(
                            self.l1_default_config
                        )  # set default values to finish lap as quickly as possible
                    rospy.wait_for_message(
                        "lap_data", LapData
                    )  # wait until lap is done
                    break  # continue with new set of parameters
                else:
                    # lap is done in a reasonable amount of time
                    if self.laps_per_experiment > 1:
                        rospy.loginfo(f"[{self.name}] {lap_count}/{self.laps_per_experiment} Laps are done")
                    y_temporary += (
                        self.calculate_cost()
                    )  # sum up y_new for calulating average
                    lap_time_avg += self.lap_time
                    avg_lat_err_avg += self.avg_lat_err

                # save shortest lap_time and smallest average lat. error measured
                if self.shortest_lap_time > self.lap_time:
                    self.shortest_lap_time = self.lap_time
                if self.smallest_avg_lat_err > self.avg_lat_err:
                    self.smallest_avg_lat_err = self.avg_lat_err

            if param_reasonable:
                # calculate averages:
                lap_time_avg = lap_time_avg / self.laps_per_experiment
                avg_lat_err_avg = avg_lat_err_avg / self.laps_per_experiment
                y_temporary = y_temporary / self.laps_per_experiment

                y_new = y_temporary
            else:
                # set cost to high value (avoid choosing the same parameters)
                y_new = high_cost_punishment
                lap_time_avg = init_lap_time
                avg_lat_err_avg = init_avg_lat_err
                self.min_car_distance = 0

            self.lap_time_list.append(lap_time_avg)
            self.avg_lat_err_list.append(avg_lat_err_avg)
            self.min_car_dist_list.append(self.min_car_distance)
            self.y_list.append(y_new)

            # store optimal value in a separate variable
            if y_new <= y_opt:
                y_opt = y_new
                self.param_opt = param_new
                self.optimal_lap_time = lap_time_avg
                self.optimal_avg_lat_err = avg_lat_err_avg
                self.optimal_min_car_dist = self.min_car_distance

            self.y_opt_list.append(y_opt)

        self.update_parameters(self.param_opt)  # set the optimal values

        rospy.loginfo(f"[{self.name}] -------------------------- R E S U L T S --------------------------")
        rospy.loginfo(f"[{self.name}] Optimizers: {np.round(self.param_opt, 4)}  ; Optimum:  {np.round(y_opt, 4)}"
        )
        rospy.loginfo(f"[{self.name}] Optimal lap_time average: {self.optimal_lap_time} \nOptimal avg_lat_err average: {self.optimal_avg_lat_err} \nOptimal min_car_dist: {self.optimal_min_car_dist}"
        )
        rospy.loginfo(f"[{self.name}] Fastest lap_time recorded: {self.shortest_lap_time} \nSmallest avg_lat_err recorded: {self.smallest_avg_lat_err}"
        )
        # TODO optimal iteration
        rospy.loginfo(f"[{self.name}] -------------------------------------------------------------------")

    #############
    # VIZ+PLOTS #
    #############
    def visualize_data(self, save_plot_bool=False, location=None):
        # for skipped laps box:
        alpha_val = 0.2
        box_color = "gray"

        # for plot saving only:
        if save_plot_bool:
            yaml_path = rospy.get_param("/bayesopt_config")
            with open(yaml_path, "r") as file:
                data = yaml.safe_load(file)
            acq_func = data["acq_func"]

        x = range(1, 1 + self.iterations)

        if self.opt_l1params:  # order: t_clip_min, m_l1, q_l1
            param_list_0 = [i[0] for i in self.param_list]
            param_list_1 = [i[1] for i in self.param_list]
            param_list_2 = [i[2] for i in self.param_list]

        # Plot the data for each graph as points
        plt.figure()
        plt.plot(
            x,
            self.y_list[1: self.iterations + 1],
            label="current cost",
            linestyle="-",
            linewidth=2,
            color="#0000FA",
        )
        plt.plot(
            x,
            self.y_opt_list[1: self.iterations + 1],
            label="optimal cost",
            linestyle=":",
            linewidth=3,
            color="#00FA00",
        )
        plt.suptitle("Bayesian Optimization: Cost Values")
        plt.title(
            f"Optimal cost: {round(self.y_opt_list[-1],3)}", fontsize=10
        )  # last element is optimal y
        plt.grid(True, linewidth=0.5, color="gray", linestyle="--")
        plt.legend()
        plt.xlabel("Iterations")
        plt.ylabel("value")
        # mark skipped iterations:
        for skipped_it in self.skipped_iterations:
            # mark a box around skipped values
            plt.axvspan(
                skipped_it - 0.5,
                skipped_it + 0.5,
                alpha=alpha_val,
                color=box_color,
                linewidth=0,
            )
        if save_plot_bool:
            plotname = "cost"
            self.save_plot(acq_func, plotname, location)

        # Plot the lap time
        plt.figure()
        plt.plot(x, self.lap_time_list, color="#9A0000")
        plt.suptitle("Bayesian Optimization: Lap_time Average")
        plt.title(
            f"Optimal lap_time average: {round(self.optimal_lap_time,3)}", fontsize=10
        )
        plt.grid(True, linewidth=0.5, color="gray", linestyle="--")
        # plt.axhline(y=self.experiment_timeout , color='black', linestyle='-', linewidth=1 , label='Timeout') # mark timeout in plot
        plt.legend()
        plt.xlabel("Iterations")
        plt.ylabel("Lap_time Average [s]")
        for skipped_it in self.skipped_iterations:
            # mark a box around skipped values
            plt.axvspan(
                skipped_it - 0.5,
                skipped_it + 0.5,
                alpha=alpha_val,
                color=box_color,
                linewidth=0,
            )
        if save_plot_bool:
            plotname = "lap_time"
            self.save_plot(acq_func, plotname, location)

        # Plot the average lateral error
        plt.figure()
        plt.plot(x, self.avg_lat_err_list, color="#FF4444")
        plt.suptitle("Bayesian Optimization: Avg_lat_error Average")
        plt.title(
            f"Optimal avg_lat_err average: {round(self.optimal_avg_lat_err,3)}",
            fontsize=10,
        )
        plt.grid(True, linewidth=0.5, color="gray", linestyle="--")
        plt.xlabel("Iterations")
        plt.ylabel("Avg_lat_err Average [m]")
        for skipped_it in self.skipped_iterations:
            # mark a box around skipped values
            plt.axvspan(
                skipped_it - 0.5,
                skipped_it + 0.5,
                alpha=alpha_val,
                color=box_color,
                linewidth=0,
            )
        if save_plot_bool:
            plotname = "avg_lat_err"
            self.save_plot(acq_func, plotname, location)

        # plot min car distance to track boundary
        plt.figure()
        plt.plot(
            x, self.min_car_dist_list, color="#FF0044"
        )  # [1:] as first value is from the first "calculate_cost" and not relevant
        plt.suptitle("Bayesian Optimization: Min dististance to boundary")
        plt.grid(True, linewidth=0.5, color="gray", linestyle="--")
        plt.xlabel("Iterations")
        plt.ylabel("Min_dist [m]")
        for skipped_it in self.skipped_iterations:
            # mark a box around skipped values
            plt.axvspan(
                skipped_it - 0.5,
                skipped_it + 0.5,
                alpha=alpha_val,
                color=box_color,
                linewidth=0,
            )
        if save_plot_bool:
            plotname = "min_dist"
            self.save_plot(acq_func, plotname, location)

        # parameter: l1 param
        if self.opt_l1params:
            plt.figure()  # on separate window
            plt.scatter(x, param_list_0, label="t_clip_min", color="#A487CA")
            plt.scatter(x, param_list_1, label="m_l1", color="#FFB347")
            plt.scatter(x, param_list_2, label="q_l1", color="#4DA6A6")
            plt.suptitle("Bayesian Optimization: Parameter Samples")
            plt.grid(True, linewidth=0.5, color="gray", linestyle="--")
            subtitle = ""  # write optimal values into subtitle
            for element in self.param_opt[
                : self.number_of_l1_param
            ]:  # round each value before putting it in subtitle
                subtitle += str(round(element, 3)) + ", "
            plt.title(f"Optimizer: {subtitle}", fontsize=10)
            plt.legend()
            plt.xlabel("Iterations")
            plt.ylabel("value")
            for skipped_it in self.skipped_iterations:
                # mark a box around skipped values
                plt.axvspan(
                    skipped_it - 0.5,
                    skipped_it + 0.5,
                    alpha=alpha_val,
                    color=box_color,
                    linewidth=0,
                )
            if save_plot_bool:
                plotname = "l1_param"
                self.save_plot(acq_func, plotname, location)

        # parameter: sector scalers
        if self.opt_sector_scalers:
            plt.figure()  # on separate window
            plt.suptitle("Bayesian Optimization: Sector Scalers")
            subtitle = ""  # write optimal values into subtitle
            for element in self.param_opt[
                self.number_of_l1_param:
            ]:  # round each value before putting it in subtitle
                subtitle += str(round(element, 3)) + ", "
            plt.title(f"Optimizer: {subtitle}", fontsize=10)
            plt.grid(True, linewidth=0.5, color="gray", linestyle="--")
            for sector_idx in range(
                self.n_sectors
            ):  # get the sector values from each sector
                sector_scalers = [
                    i[sector_idx + (self.number_of_l1_param)] for i in self.param_list
                ]  # get array of sectors of each iteration
                plt.scatter(
                    x,
                    sector_scalers,
                    label="Sector" + str(sector_idx),
                    alpha=1.0 / (sector_idx + 1),
                    color="black",
                )  # plot all sector scalers (of each iteration) of current sector (e.g. sector 0)
            plt.legend()
            plt.xlabel("Iterations")
            plt.ylabel("value")
            if save_plot_bool:
                plotname = "sectors"
                self.save_plot(acq_func, plotname, location)

        plt.show()

    def save_plot(self, acq_func, plotname, location):
        filename = acq_func + "_" + plotname + "_"
        filename += str(self.iterations) + "ite_"
        filename += str(self.laps_per_experiment) + "lpe_"
        filename += ".png"

        home_dir = os.path.expanduser("~")  # Get the user's home directory
        path = os.path.join(home_dir, location, filename)
        plt.savefig(path)

    def get_racecar_version(self):
        racecar_version = rospy.get_param("/racecar_version")  # NUCX or SIM
        return racecar_version


if __name__ == "__main__":
    name = "param_optimizer"
    rospy.init_node(name, anonymous=False, log_level=rospy.WARN)
    bayesopt = BayesOptimizer(name)
    rospy.spin()
