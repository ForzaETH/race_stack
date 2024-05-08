from __future__ import annotations
import os
import yaml
from typing import List, TYPE_CHECKING
from rclpy.node import Node
from dataclasses import dataclass
from state_machine.state_types import StateType
from rcl_interfaces.msg import ParameterDescriptor, SetParametersResult, Parameter, ParameterType, FloatingPointRange, IntegerRange, ParameterValue
from ament_index_python import get_package_share_directory

if TYPE_CHECKING:
    from state_machine.state_machine import StateMachine

class StateMachineParams:
    def __init__(self, node: StateMachine) -> None:
        self.node = node
        self.package_name = node.get_name()
        
        # get stack_master package path
        stack_master_path = get_package_share_directory('stack_master')
        # get config path of state_machine_params.yaml
        config_path = os.path.join(stack_master_path, 'config', 'state_machine_params.yaml')
        with open(config_path, 'r') as f:
            self.param_dict = yaml.safe_load(f)
            self.param_dict = self.param_dict['state_machine']['ros__parameters']

        descriptor = ParameterDescriptor(
            description="Rate at which the state machine should run in Hz\n",
            read_only=False,
            type=ParameterType.PARAMETER_INTEGER,
            integer_range=[IntegerRange(from_value=10, to_value=100, step=1)]
        )
        node.set_descriptor("rate_hz", descriptor=descriptor)
        self.rate_hz: int = node.get_parameter("rate_hz").value
        
        descriptor = ParameterDescriptor(
            description=f"Initial state of the state machine\n One can choose between {[str(state) for state in StateType]}\n",
            type=ParameterType.PARAMETER_STRING,
            read_only=False,
        )
        node.set_descriptor("initial_state", descriptor=descriptor)
        self.initial_state: str = node.get_parameter("initial_state").value
        
        descriptor = ParameterDescriptor(
            description="Flag to enable testing on car\nSet to true also when testing on a rosbag\n",
            type=ParameterType.PARAMETER_BOOL,
            read_only=False,
        )
        node.set_descriptor("test_on_car", descriptor=descriptor)
        self.test_on_car: bool = node.get_parameter("test_on_car").value
        
        descriptor = ParameterDescriptor(
            description="Mode of the state machine\nChoose between dummy, time_trials, and head_to_head\n",
            type=ParameterType.PARAMETER_STRING,
            read_only=False,
        )
        node.set_descriptor("mode", descriptor=descriptor)
        self.mode: str = node.get_parameter("mode").value
        
        descriptor = ParameterDescriptor(
            description="Number of local waypoints\n",
            type=ParameterType.PARAMETER_INTEGER,
            read_only=False,
            integer_range=[IntegerRange(from_value=40, to_value=200, step=5)]
        )
        node.set_descriptor("n_loc_wpnts", descriptor=descriptor)
        self.n_loc_wpnts: int = node.get_parameter("n_loc_wpnts").value
        
        self.overtake_mode = "spliner"
        """Overtake mode\nOnly spliner is supported at the moment"""
        
        descriptor = ParameterDescriptor(
            description="Voltage threshold for the car, below which the car is considered to be low bat.\n",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(
                from_value=9.0,
                to_value=15.0,
                step=0.5)])
        node.set_descriptor("volt_threshold", descriptor=descriptor)
        self.volt_threshold: float = node.get_parameter("volt_threshold").value

        descriptor = ParameterDescriptor(
            description="Flag to force the state machine to be in a particular state\n",
            type=ParameterType.PARAMETER_BOOL,
            read_only=False,
        )
        node.set_descriptor("force_state", descriptor=descriptor)
        self.force_state: bool = node.get_parameter("force_state").value
        
        # this is actual a parameter not defined in the yaml (no reason for it)
        self.force_state_choice: str = StateType.GB_TRACK
        """State in which the state machine should be forced to be in, in case the force_state parameter is set to True"""
        descriptor = ParameterDescriptor(
            type=ParameterType.PARAMETER_STRING,
            description=f"State in which the state machine should be forced to be in, in case the force_state parameter is set to True\n One can choose between {[str(state) for state in StateType]}\n"
        )
        node.declare_parameter("force_state_choice", self.force_state_choice.value, descriptor=descriptor)

        descriptor = ParameterDescriptor(
            description="Time to live for splini\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=1.0,
                to_value=5.0,
                step=0.1)])
        node.set_descriptor("splini_ttl", descriptor=descriptor)
        self.splini_ttl: int = node.get_parameter("splini_ttl").value
        
        descriptor = ParameterDescriptor(
            description="Distance from gb path for rejoining in meters\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=5.0,
                step=0.1)])
        node.set_descriptor("gb_ego_width_m", descriptor=descriptor)
        self.gb_ego_width_m: float = node.get_parameter("gb_ego_width_m").value

        descriptor = ParameterDescriptor(
            description="lateral width within which we consider obstacles to be on the overtaking line in meters\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=5.0,
                step=0.1)]
        )
        node.set_descriptor("lateral_width_gb_m", descriptor=descriptor)
        self.lateral_width_gb_m: float = node.get_parameter("lateral_width_gb_m").value
        
        descriptor = ParameterDescriptor(
            description="horizon considered for the global waypoints to check for obstacles in meters\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=3.0,
                to_value=20.0,
                step=0.1)]
            )
        node.set_descriptor("gb_horizon_m", descriptor=descriptor)
        self.gb_horizon_m: float = node.get_parameter("gb_horizon_m").value
        
        descriptor = ParameterDescriptor(
            description="horizon considered for overtaking in meters\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=3.0,
                to_value=20.0,
                step=0.1)]
        )
        node.set_descriptor("overtaking_horizon_m", descriptor=descriptor)
        self.overtaking_horizon_m: float = node.get_parameter("overtaking_horizon_m").value
        
        descriptor = ParameterDescriptor(
            description="lateral width within which we consider opponents to be on the overtaking line in meters\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=5.0,
                step=0.1)]
        )
        node.set_descriptor("lateral_width_ot_m", descriptor=descriptor)
        self.lateral_width_ot_m: float = node.get_parameter("lateral_width_ot_m").value
        
        descriptor = ParameterDescriptor(
            description="time we have to wait between switching from overtaking on one side to the other in seconds\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=2.0,
                step=0.1)]
        )
        node.set_descriptor("splini_hyst_timer_sec", descriptor=descriptor)
        self.splini_hyst_timer_sec: float = node.get_parameter("splini_hyst_timer_sec").value
        
        descriptor = ParameterDescriptor(
            description="time we have to stay slower than the ftg_threshold_speed threshold before we activate FTG in seconds\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=1.0,
                to_value=5.0,
                step=0.1)]
        )
        node.set_descriptor("ftg_timer_sec", descriptor=descriptor)
        self.ftg_timer_sec: float = node.get_parameter("ftg_timer_sec").value
        
        descriptor = ParameterDescriptor(
            description="speed threshold below which we start a timer for activating FTG in m/s\n",
            read_only=False,
            type=ParameterType.PARAMETER_DOUBLE,
            floating_point_range=[FloatingPointRange(
                from_value=0.0,
                to_value=5.0,
                step=0.1)]
        )
        node.set_descriptor("ftg_threshold_speed", descriptor=descriptor)
        self.ftg_threshold_speed: float = node.get_parameter("ftg_threshold_speed").value

    def parameters_callback(self, parameters: List[Parameter]) -> SetParametersResult:
        for param in parameters:
            match param.name:
                case "rate_hz":
                    self.node.main_loop.timer_period_ns = int(1e9 / param.value)
                case "initial_state":
                    self.node.state = StateType(param.value)
                case "force_state_choice":
                    try:
                        self.force_state_choice = StateType(param.value)
                    except ValueError:
                        self.node.get_logger().error(f"Invalid state '{param.value}'")
                        return SetParametersResult(successful=False)
                case _:
                    self.__dict__[param.name] = param.value
            
            self.node.get_logger().info(f"Parameter '{param.name}' was set to {param.value}")

        return SetParametersResult(successful=True)