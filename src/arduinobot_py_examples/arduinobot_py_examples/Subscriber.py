import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class simpleSub(Node):
    def __init__(self):
        super().__init__("simple_sub")
        self.sub = self.create_subscription(String,"chatter",self.msgcallback,10)

    def msgcallback(self,msg):
        self.get_logger().info("I heard %s"%msg.data)

def main():
    rclpy.init()
    simple_Sub = simpleSub()
    rclpy.spin(simple_Sub)
    simple_Sub.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()