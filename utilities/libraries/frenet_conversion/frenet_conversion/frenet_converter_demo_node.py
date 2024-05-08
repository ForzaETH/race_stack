
import rclpy
from rclpy.node import Node
from frenet_conversion.frenet_converter import FrenetConverter
import random

import numpy as np
from f110_msgs.msg import Wpnt


class FrenetConverterDemo(Node):
    """
    A little demo node that shows how the FrenetConverter can be imported and used.
    """

    def __init__(self):
        super().__init__('Frenet_Converter_Demo_Node')
        self.publisher_ = self.create_publisher(Wpnt, 'frenet_wpnts', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.converter = FrenetConverter(np.array([0,1,1,0]),np.array([1,1,0,0]))
        self.i = 0

    def timer_callback(self):
        msg = Wpnt()
        msg.id = self.i
        x = [random.random()*1.5-0.25]
        y = [random.random()*1.5-0.25]
        msg.x_m = x[0]
        msg.y_m = y[0]
        sd = self.converter.get_frenet(x,y)
        msg.s_m = sd[0,0]
        msg.d_m = sd[1,0]
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing Frenet conversion for ID: "%d"' % msg.id)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    frenet_converter_demo = FrenetConverterDemo()

    rclpy.spin(frenet_converter_demo)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    frenet_converter_demo.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
