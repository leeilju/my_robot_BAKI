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

# from std_msgs.msg import String
from my_interfaces.msg import Numbers


import sys
sys.path.append("/home/iljujjang/catkin_ws/src/BAKI/ASTAR")

import astar



class MinimalPublisher(Node): #Node로 붙어 산속받아 만들어진 클래스

    # def __init__(self):
    #     super().__init__('minimal_publisher')  # 만들고자 하는 노드의 이름 삽입
    #     self.publisher_ = self.create_publisher(String, 'topic', 10) #전달인자 입력, 10 = sub가 충분히 받지 못할경우 입력해 놓을 quesize
    #     timer_period = 0.01  # seconds
    #     self.timer = self.create_timer(timer_period, self.timer_callback)
    #     self.i = 0 # callback을 위한 변수

    # def timer_callback(self):
    #     msg = String()
    #     msg.data = 'Hello World: %d' % self.i    # %는 대입연산자를 의미함.
    #     self.publisher_.publish(msg)
    #     self.get_logger().info('Publishing: "%s"' % msg.data)
    #     self.i += 1
    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(Numbers, 'topic', 10)     # CHANGE
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i1 = []
        self.i2 = []
        self.listlength = None
        

    def timer_callback(self):

        gx = 50.0  # [m]
        gy = 52.0  # [m]
        grid_size = 2.0  # [m]
        robot_radius = 1.5  # [m]
        
        ox, oy = [], []
        for i in range(-10, 60):
            ox.append(i)
            oy.append(-10.0)
        for i in range(-10, 60):
            ox.append(60.0)
            oy.append(i)
        for i in range(-10, 61):
            ox.append(i)
            oy.append(60.0)
        for i in range(-10, 61):
            ox.append(-10.0)
            oy.append(i)
        for i in range(-10, 40):
            ox.append(20.0)
            oy.append(i)
        for i in range(0, 40):
            ox.append(40.0)
            oy.append(60.0 - i)
        for i in range(45,55):
            ox.append(48.0)
            oy.append(i)
        for i in range(45,55):
            ox.append(52.0)
            oy.append(i)

        a_star = astar.AStarPlanner(ox, oy, grid_size, robot_radius)
    
        cx_path, cy_path = a_star.planning(0,0,gx,gy)
        self.i1 = cx_path[::-1]
        self.i2 = cy_path[::-1]

        print(self.i1)
        print(self.i2)

        self.listlength = len(self.i1)



        msg = Numbers()                                 # CHANGE
        msg.a = self.i1
        msg.b = self.i2

        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%d | %d "' % (msg.a[1],msg.b[2]))  # CHANGE

def main(args=None):
    rclpy.init(args=args) # rclpy.init이 사용되는순간 node가 생성된다. 

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
