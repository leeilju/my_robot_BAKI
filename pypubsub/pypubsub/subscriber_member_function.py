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
        self.publisher_= self.create_publisher(Numbers, 'topic2', 10)
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.listlength = None
        
        self.subscription
        self.count = 0
        self.msg2 = Numbers()


    def listener_callback(self, msg):
            # self.get_logger().info('I heard: "%d  | %d  "' % (msg.a[1],msg.b[1])) # CHANGE
            for i in range (len(msg.a)):
                 print('%d : x%d=%f, y%d=%f'%(self.count, i,msg.a[i],i,msg.b[i]))
            self.count = self.count + 1
            self.msg2.a = msg.a
            self.msg2.b = msg.b

    def timer_callback(self):
         msg = Numbers()
         msg = self.msg2
         self.publisher_.publish(msg)

        


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
