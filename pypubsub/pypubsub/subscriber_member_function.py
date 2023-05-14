import rclpy
from rclpy.node import Node

# from std_msgs.msg import String
from my_interfaces.msg import Numbers


# class MinimalSubscriber(Node):

#     def __init__(self):
#         super().__init__('minimal_subscriber')
#         self.subscription = self.create_subscription(
#             String,
#             'topic',
#             self.listener_callback,
#             10)
#         self.subscription  # prevent unused variable warning

#     def listener_callback(self, msg):
#         self.get_logger().info('I heard: "%s"' % msg.data) #sting "hello world %d" % self.i를 전달받음


class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Numbers,                                              # CHANGE
            'topic',
            self.listener_callback,
            10)
        self.subscription

    def listener_callback(self, msg):
            # self.get_logger().info('I heard: "%d  | %d  "' % (msg.a[1],msg.b[1])) # CHANGE
            for i in range (len(msg.a)):
                 print('x%d=%f, y%d=%f'%(i,msg.a[i],i,msg.b[i]))



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
