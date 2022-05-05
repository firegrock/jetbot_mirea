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

from jetbot import Robot

from std_msgs.msg import String
from std_msgs.msg import Int32

class MinimalSubscriber(Node):

    left_speed = 0
    right_speed = 0
    robot = Robot()

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(Int32, '/lspeed', self.listener_l_callback, 10)
        self.subscription = self.create_subscription(Int32, '/rspeed', self.listener_r_callback, 10)
        self.subscription  # prevent unused variable warning

        

    def listener_l_callback(self, msg):
        self.left_speed = msg.data
        self.get_logger().info('I heard left speed: "%i"' % msg.data)
        
        self.robot.set_motors(self.left_speed/100, self.right_speed/100)
    
    def listener_r_callback(self, msg):
        self.right_speed = msg.data
        self.get_logger().info('I heard right speed: "%i"' % msg.data)

        self.robot.set_motors(self.left_speed/100, self.right_speed/100)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    
    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()