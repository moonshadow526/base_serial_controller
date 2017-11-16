//
// Created by chke on 7/14/17.
//
#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <nav_msgs/Odometry.h>
#include <tf/transform_broadcaster.h>
#include <sensor_msgs/JointState.h>
#include <boost/assign.hpp>

#include "serila_controller/serial_data_interactive.h"
#include "serila_controller/robot_fun_state.h"

#include "../include/serial.h"
#include "../include/serial_control.h"
#include "../include/serial_server_process.h"
#include "../include/cacl_odom.h"

extern  ROBOT_PARA robot_odom;

//static long int left_encoder_count = 0;
//static long int right_encoder_count = 0;
//static long int lspeed_encoder_count = 0;
//static long int rspeed_encoder_count = 0;
//
//int rcallbackcount = 0;
//float left_speed_cmd = 0.0;
//float right_speed_cmd = 0.0;
extern float left_speed;
extern float right_speed;

//位置
extern float px;
extern float py;
extern float theta;

//速度

extern float vx;
extern float vth;

pthread_rwlock_t rwlock = PTHREAD_RWLOCK_INITIALIZER;

bool serial_data_server(serila_controller::serial_data_interactive::Request &req, serila_controller::serial_data_interactive::Response &res)
{
    return serial_server_process(req, res);
}

int main(int argc, char ** argv)
{

    int fd = 0;
    robot_odom.odom->right_odom = 0;
    robot_odom.odom->left_odom = 0;

    SerialControl serialControl;

    fd = serialControl.open_serial_port(1);
    if (0 == fd){
        serialControl.read_serial_task();
    }

//    Control.close_encoder_port();
    else{
        while (-1 == fd)
        {
            fd = serialControl.open_serial_port(1);
            sleep(1);
        }
        serialControl.read_serial_task();
    }
//    while (1);
    ros::Time current_time;

    ros::init(argc, argv, "serial_control_node");
    ros::NodeHandle n;

    tf::TransformBroadcaster odom_broadcaster;
    bool publish_tf;
    n.param("publish_tf", publish_tf, true);

    nav_msgs::Odometry odom;
    sensor_msgs::JointState joint_state;
    joint_state.name.resize(2);
    joint_state.position.resize(2);
    joint_state.velocity.resize(2);
    joint_state.name[0] = "left_wheel_joint";
    joint_state.name[1] = "right_wheel_joint";

    ros::Publisher odom_pub = n.advertise<nav_msgs::Odometry>("/odom", 10);
    ros::Publisher joint_state_pub = n.advertise<sensor_msgs::JointState>("/joint_states", 10);
    ros::Publisher robot_fun_state_pub = n.advertise<serila_controller::robot_fun_state>("/robot_fun_state", 10);
    ros::Subscriber cmd_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 1, cmd_vel_set);

    ros::ServiceServer service = n.advertiseService("serial_data_server",serial_data_server);
    ROS_INFO("server ready");

    ros::Rate r(50);
    while (ros::ok())
    {
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(theta);
        current_time = ros::Time::now();
        calc_odom();
//        left_speed = robot_odom.odom->left_odom/20;
//        right_speed = robot_odom.odom->right_odom/20;

        if(publish_tf)
        {
            geometry_msgs::TransformStamped odom_trans;
            odom_trans.header.stamp = current_time;
            odom_trans.header.frame_id = "odom";
            odom_trans.child_frame_id = "base_footprint";

            odom_trans.transform.translation.x = px;
            odom_trans.transform.translation.y = py;
            odom_trans.transform.translation.z = 0.0;
            odom_trans.transform.rotation = odom_quat;

            odom_broadcaster.sendTransform(odom_trans);
        }

        odom.header.stamp = current_time;
        odom.header.frame_id = "odom";
        odom.child_frame_id = "base_footprint";

        odom.pose.pose.position.x = px;
        odom.pose.pose.position.y = py;
        odom.pose.pose.position.z = 0.0;
        odom.pose.pose.orientation = odom_quat;
        odom.pose.covariance =  boost::assign::list_of (1e-3)  (0) (0)  (0)  (0)  (0)
                (0) (1e-3)  (0)  (0)  (0)  (0)
                (0)   (0)  (1e6) (0)  (0)  (0)
                (0)   (0)   (0) (1e6) (0)  (0)
                (0)   (0)   (0)  (0) (1e6) (0)
                (0)   (0)   (0)  (0)  (0)  (1);
        odom.twist.twist.linear.x = vx;
        odom.twist.twist.linear.y = 0;
        odom.twist.twist.angular.z = vth;
        odom.twist.covariance =  boost::assign::list_of (1e-3)  (0) (0)  (0)  (0)  (0)
                (0) (1e-3)  (0)  (0)  (0)  (0)
                (0)   (0)  (1e6) (0)  (0)  (0)
                (0)   (0)   (0) (1e6) (0)  (0)
                (0)   (0)   (0)  (0) (1e6) (0)
                (0)   (0)   (0)  (0)  (0)  (1);
        odom_pub.publish(odom);

        joint_state.header.stamp = current_time;
        joint_state.position[0] = robot_odom.odom->left_odom / TICKSPERRADIAN;      //m
        joint_state.position[1] = robot_odom.odom->right_odom / TICKSPERRADIAN;     //m
        joint_state.velocity[0] = left_speed;
        joint_state.velocity[1] = right_speed;
        joint_state_pub.publish(joint_state);

        if (WaterSeat.pub_flag)
        {
            serila_controller::robot_fun_state waterSeat;
            waterSeat.device_name = "waret_seat";
            waterSeat.para = WaterSeat.info_byte[0];
            robot_fun_state_pub.publish(waterSeat);

        }

        if (DoorState.pub_flag)
        {
            serila_controller::robot_fun_state doorState;
            doorState.device_name = "doorstate";
            doorState.para = WaterSeat.info_byte[0];
            robot_fun_state_pub.publish(doorState);

        }
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}