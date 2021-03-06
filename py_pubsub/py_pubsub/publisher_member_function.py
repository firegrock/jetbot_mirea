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

from torch import int32
import rclpy
from rclpy.node import Node

from std_msgs.msg import String
from std_msgs.msg import Int32


class MinimalPublisher(Node):
    left_speed = 0
    right_speed = 0
    speed_step = 1
    i = 0

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher1_ = self.create_publisher(Int32, 'lspeed', 10)
        self.publisher2_ = self.create_publisher(Int32, 'rspeed', 10)
        
        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg1 = Int32()
        msg2 = Int32()
        
        if self.i <= 20:
            msg1.data = 60 - 5
            msg2.data = 60
        elif self.i > 20 and self.i <= 23:
            msg1.data = -60 
            msg2.data = 60
        elif self.i > 24 and self.i <= 45:
            msg1.data = 60 - 5
            msg2.data = 60
        elif self.i > 45 and self.i <= 48:
            msg1.data = -60
            msg2.data = 60
        else:
            self.i = 0
        
        self.i = self.i+1
        
        self.publisher1_.publish(msg1)
        self.publisher2_.publish(msg2)
        #self.get_logger().info('Publishing: "%i" "%i' % msg1.data, msg2.data)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()