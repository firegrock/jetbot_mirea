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

import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32
from std_msgs.msg import Float32
from std_msgs.msg import Bool


class SpeedFeedback_Publisher(Node):
    real_speed = 0.0

    def __init__(self):
        super().__init__('minimal_publisher')
        
        self.publisher1_ = self.create_publisher(Int32, 'real_lspeed', 10)

        self.subscription = self.create_subscription(Bool, '/left_enc', self.listener_callback, 10)
        self.subscription  # prevent unused variable warning
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        encoder_max_impulse_discriptor = ParameterDescriptor(description='This parameter is defined the maximum number of encoder' + "'" + 's impulses during measure period')
        self.declare_parameter('max_impulses', '120', encoder_max_impulse_discriptor)

        encoder_topic_name = ParameterDescriptor(description = 'Name of the topic where current state of fc_33 encoder is publishing. The default name is /left_enc')
        self.declare_parameter('encoder_node_name', '/left_enc', encoder_topic_name)

    def timer_callback(self):
        msg = Float32()
        
        msg.data = 0
        
        self.publisher_.publish(msg)

        #self.get_logger().info('Publishing: "%i" "%i' % msg1.data, msg2.data)


def main(args=None):
    rclpy.init(args=args)

    real_speed_publisher = SpeedFeedback_Publisher()

    rclpy.spin(real_speed_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    GPIO.cleanup()  # cleanup all GPIOs
    real_speed_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()