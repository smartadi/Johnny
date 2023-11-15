#include <ros.h>
#include <std_msgs/String.h>
#include <std_msgs/UInt16.h>

ros::NodeHandle nh;
std_msgs::UInt16 sensor_msg;
ros::Publisher sensor_publisher("sensor_data", &sensor_msg);

void setup() {
  nh.initNode();
  nh.advertise(sensor_publisher);
}

void loop() {
  sensor_msg.data = analogRead(A0);
  sensor_publisher.publish(&sensor_msg);
  nh.spinOnce();
  delay(1);
}