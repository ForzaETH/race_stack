import os
import yaml
import rclpy
import rclpy.node
from rcl_interfaces.msg import SetParametersResult
from rclpy.parameter import Parameter

from typing import List

class GlobalParams(rclpy.node.Node):
    def __init__(self):
        super().__init__('global_parameters',
                         allow_undeclared_parameters=True,
                         automatically_declare_parameters_from_overrides=True)

        self.add_on_set_parameters_callback(self.parameter_change_callback)

    def parameter_change_callback(self, params: List[Parameter]) -> SetParametersResult:
        """Gets called whenever there is an attempt to change one or more parameters.

        Args:
            params (List[Parameter]): A list of Parameter objects representing the parameters that are 
                being attempted to change.

        Returns:
            SetParametersResult: Object indicating whether the change was successful.
        """

        # Iterate over each parameter in this node
        for param in params:
            self.get_logger().warn(f"{param.name} [{param.type_}]: {param.value}")
        return SetParametersResult(successful=True)


def main():
    rclpy.init()
    node = GlobalParams()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()
