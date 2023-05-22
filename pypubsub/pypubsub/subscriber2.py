import rclpy
from rclpy.node import Node

from my_interfaces.msg import Numbers

class Sub2(Node):

    def __init__(self):
        super().__init__('subscribe2')
        self.subscription = self.create_subscription(
            Numbers,
            'topic2',
            self.listener_callback,
            10
        )
        self.subscription
        self.count = 0

    def listener_callback(self, msg):
        for i in range(len(msg.a)):
            print('%d : x%d=%f, y%d=%f'%(self.count, i,msg.a[i],i,msg.b[i]))
        self.count = self.count +1

def main(args=None):
    rclpy.init(args=args)

    subscribe2 = Sub2()
    rclpy.spin(subscribe2)

    subscribe2.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
