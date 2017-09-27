/*
 * gyroscope_tf.cpp
 *
 *  Created on: Jun 8, 2017
 *      Author: Enrique
 *  DO NOT USE, DEPRECATED
 */
 ///INCLUDES
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/Imu.h>

///Variables
geometry_msgs::Quaternion gyroscope_quaternion;
std::string gyroscope_frame_name = "gyroscope";
//ROS publishers
ros::Publisher pub;

///FUNCTIONS

//IMU callback function for getting gyroscope quaternion
void imuCallback(const sensor_msgs::ImuConstPtr& msg)
{
  static tf::TransformBroadcaster gyro_br;
  tf::Transform transform;
  transform.setOrigin( tf::Vector3(0.0, 0.0, 0.0) );
  gyroscope_quaternion = msg -> orientation;
  tf::Quaternion q;
  quaternionMsgToTF(gyroscope_quaternion, q); //convert between geometry_msgs::Quaternion and tf::Quaternion
  transform.setRotation(q);
  gyro_br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "world", gyroscope_frame_name));
  pub.publish(gyroscope_quaternion); //publish quaternion
}

///MAIN LOOP
int main(int argc, char** argv)
{
  ros::init(argc, argv, "gyroscope_tf");
    ros::NodeHandle nh;
    ros::Subscriber imu_sub = nh.subscribe("/imu", 1, imuCallback); // /imu topic from razor_9df_imu node
    pub = nh.advertise<geometry_msgs::Quaternion>("gyroscope/imu_quaternion", 1);
    ros::spin();
    return 0;
}
