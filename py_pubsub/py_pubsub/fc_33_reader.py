# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Bool

from rcl_interfaces.msg import ParameterDescriptor

import RPi.GPIO as GPIO


class Fc_33Publisher(Node):

    def __init__(self):
        super().__init__('fc_33_readings_publisher')
        

        fc_33_pin_descriptor = ParameterDescriptor(description='This parameter is defined the gpio' + "'" + 's pin number to which fc-33 is connected!')
        self.declare_parameter('fc_33_pin', '12', fc_33_pin_descriptor)

        fc_33_node_name_discriptor = ParameterDescriptor(description='This parameter is defined the name of node that is going to be publishing info')
        self.declare_parameter('fc_33_node_name', 'fc_33_readings', fc_33_pin_descriptor)

        self.publisher_ = self.create_publisher(Bool, self.get_parameter('fc_33_node_name').get_parameter_value().string_value, 10)
        
        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)


    def timer_callback(self):
        fc_33_pin = int(self.get_parameter('fc_33_pin').get_parameter_value().string_value)
        
        GPIO.setmode(GPIO.BOARD)
        GPIO.setup(fc_33_pin, GPIO.IN, pull_up_down=GPIO.PUD_UP)

        msg = Bool()

        msg.data = bool(GPIO.input(fc_33_pin))
        
        self.publisher_.publish(msg)

        #self.get_logger().info('Publishing: "%i" "%i' % msg1.data, msg2.data)


def main(args=None):
    rclpy.init(args=args)

    fc_33_publisher = Fc_33Publisher()

    rclpy.spin(fc_33_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    fc_33_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()