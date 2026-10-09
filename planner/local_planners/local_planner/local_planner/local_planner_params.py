from __future__ import annotations
from typing import TYPE_CHECKING
from rcl_interfaces.msg import ParameterDescriptor, ParameterType, FloatingPointRange, IntegerRange

if TYPE_CHECKING:
    from local_planner.local_planner import LocalPlanner


class LocalPlannerParams:
    def __init__(self, node: LocalPlanner) -> None:
        self.node = node

        descriptor = ParameterDescriptor(
            description="Rate at which the local planner should run in Hz\n",
            type=ParameterType.PARAMETER_INTEGER,
            read_only=False,
            integer_range=[IntegerRange(from_value=10, to_value=100, step=1)],
        )
        node.set_descriptor("rate_hz", descriptor=descriptor)
        self.rate_hz: int = node.get_parameter("rate_hz").value

        descriptor = ParameterDescriptor(
            description="Number of local waypoints published, 1 waypoint every 0.1 meter\n",
            type=ParameterType.PARAMETER_INTEGER,
            read_only=False,
            integer_range=[IntegerRange(from_value=40, to_value=200, step=5)],
        )
        node.set_descriptor("n_loc_wpnts", descriptor=descriptor)
        self.n_loc_wpnts: int = node.get_parameter("n_loc_wpnts").value

        descriptor = ParameterDescriptor(
            description="Overtaking planner to use\nChoose between frenet, spliner, predictive_spliner, graph_based\n",
            type=ParameterType.PARAMETER_STRING,
            read_only=False,
        )
        node.set_descriptor("ot_planner", descriptor=descriptor)
        self.ot_planner: str = node.get_parameter("ot_planner").value

        descriptor = ParameterDescriptor(
            description="Distance from gb path for rejoining in meters\nNOTE: dynamically overridden from the state_machine node\n",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=5.0, step=0.1)],
        )
        node.set_descriptor("gb_ego_width_m", descriptor=descriptor)
        self.gb_ego_width_m: float = node.get_parameter("gb_ego_width_m").value

        descriptor = ParameterDescriptor(
            description="Time to live for spliner waypoints in seconds\nNOTE: dynamically overridden from the state_machine node\n",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=1.0, to_value=5.0, step=0.1)],
        )
        node.set_descriptor("splini_ttl", descriptor=descriptor)
        self.splini_ttl: float = node.get_parameter("splini_ttl").value

        descriptor = ParameterDescriptor(
            description="Time to live for predictive spliner waypoints in seconds\n",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.05)],
        )
        node.set_descriptor("pred_splini_ttl", descriptor=descriptor)
        self.pred_splini_ttl: float = node.get_parameter("pred_splini_ttl").value

        descriptor = ParameterDescriptor(
            description="Time we have to wait between switching from overtaking on one side to the other in seconds\nNOTE: dynamically overridden from the state_machine node\n",
            type=ParameterType.PARAMETER_DOUBLE,
            read_only=False,
            floating_point_range=[FloatingPointRange(from_value=0.0, to_value=2.0, step=0.1)],
        )
        node.set_descriptor("splini_hyst_timer_sec", descriptor=descriptor)
        self.splini_hyst_timer_sec: float = node.get_parameter("splini_hyst_timer_sec").value
