import rclpy
from rclpy.node import Node

from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry

import math

from .johnnyControl import Server


class JohnnyBridge(Node):

    def __init__(self):
        super().__init__('johnny_bridge')

        self.johnny_server = Server()
        print(self.johnny_server.subjectNames)

        self.publishersDict = {}
        self.subscribersDict = {}
        for i in self.johnny_server.subjectNames:
            self.publishersDict[i] = self.create_publisher(Odometry, i+'/odom', 10)
            self.subscribersDict[i] = self.create_subscription(
                TwistStamped,
                i+'/cmd_vel',
                self.cmdVelSubscriber,
                1)

        print(self.publishersDict)
        print(self.johnny_server.mover)


        #self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 0.01  # seconds
        self.lastTime = self.get_clock().now()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

    def cmdVelSubscriber(self, msg):

        johnny = msg.header.frame_id
        self.johnny_server.ref[johnny][0][0] = 1000*msg.twist.linear.x
        self.johnny_server.ref[johnny][0][1] = 1000*msg.twist.linear.y
        self.johnny_server.ref[johnny][0][2] = 1000*msg.twist.linear.z
        self.johnny_server.ref[johnny][1][0] = msg.twist.angular.x
        self.johnny_server.ref[johnny][1][1] = msg.twist.angular.y
        self.johnny_server.ref[johnny][1][2] = msg.twist.angular.z
        #print( self.johnny_server.ref)
        #print(msg.twist.linear.x)

    def timer_callback(self):
        currTime = self.get_clock().now()
        deltaT = (currTime - self.lastTime).nanoseconds
        self.lastTime = currTime
        #print(1000000000/deltaT)

        self.johnny_server.johnny_update()
        self.johnny_server.send_data()

        for i in self.johnny_server.subjectNames:
            msg = Odometry()
            msg.pose.pose.position.x = self.johnny_server.mover[i][0,0]/1000.0
            msg.pose.pose.position.y = self.johnny_server.mover[i][0,1]/1000.0
            msg.pose.pose.position.z = self.johnny_server.mover[i][0,2]/1000.0

            msg.pose.pose.orientation.w = math.cos(self.johnny_server.mover[i][1,2]/2)
            msg.pose.pose.orientation.z = math.sin(self.johnny_server.mover[i][1,2]/2)
            
            msg.twist.twist.linear.x = self.johnny_server.mover[i][2,0]/1000.0
            msg.twist.twist.linear.y = self.johnny_server.mover[i][2,1]/1000.0
            msg.twist.twist.linear.z = self.johnny_server.mover[i][2,2]/1000.0
            
            msg.twist.twist.angular.x = self.johnny_server.mover[i][3,0]
            msg.twist.twist.angular.y = self.johnny_server.mover[i][3,1]
            msg.twist.twist.angular.z = self.johnny_server.mover[i][3,2]
            self.publishersDict[i].publish(msg)
        #print(self.johnny_server.ref)


def main(args=None):
    rclpy.init(args=args)

    johnny_bridge = JohnnyBridge()

    rclpy.spin(johnny_bridge)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    johnny_bridge.destroy_node()
    rclpy.shutdown()
    print("bye bye")


if __name__ == '__main__':
    main()