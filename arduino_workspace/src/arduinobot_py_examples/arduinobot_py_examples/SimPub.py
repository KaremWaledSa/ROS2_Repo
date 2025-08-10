import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class simplePub(Node):
    def __init__(self):
        super().__init__("simple_pub")
        self.pub = self.create_publisher(String,"chatter", 10)#impor this method from rclpy.node class
        self.counter = 0
        self.freq = 1.0
        self.get_logger().info("publishing at %d hz"%self.freq)
        self.timer = self.create_timer(self.freq, self.timercallback)
    
    def timercallback(self):
        msg = String()
        msg.data = "hello world %d"%self.counter
        self.pub.publish(msg)
        self.counter += 1

def main():
    rclpy.init()
    simple_Pub = simplePub()
    rclpy.spin(simple_Pub)
    rclpy.shutdown()

if __name__ == "__main__":
    main()