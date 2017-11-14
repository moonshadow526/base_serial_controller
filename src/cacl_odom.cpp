//
// Created by chke on 11/11/17.
//

#include "../include/cacl_odom.h"
#include "../include/serial_control.h"
#include <math.h>

long int last_left_encoder_count = 0;
long int last_right_encoder_count = 0;

//rad/s
float left_speed = 0.0;
float right_speed = 0.0;

extern  ROBOT_PARA robot_odom;

//位置
float px = 0.0;
float py = 0.0;
float theta = 0.0;

//speed
float vx = 0.0;
float vth = 0.0;

void calc_odom(void)
{
    float  dt = 0.02;
    printf("right odom %ld\n",robot_odom.odom->right_odom);
    printf("left odom %ld\n",robot_odom.odom->left_odom);
    float distance_left = (robot_odom.odom->left_odom - last_left_encoder_count)/TICKSPERMETER;
    float distance_right = (robot_odom.odom->right_odom - last_right_encoder_count)/TICKSPERMETER;

    left_speed = (robot_odom.odom->left_odom - last_left_encoder_count)/TICKSPERRADIAN/dt;
    right_speed = (robot_odom.odom->right_odom - last_right_encoder_count)/TICKSPERRADIAN/dt;

    printf("\n\nleft speed of rad/s %f\n\n", left_speed);
    printf("\n\nright speed of rad/s %f\n\n", right_speed);

    float dis_xy = (distance_left + distance_right)/2;
    float dth = (distance_right - distance_left)/WHEEL_SEPARATION;
    vx = dis_xy /dt;

    vth = dth /dt;
    if(dis_xy != 0)
    {
        float dx = cos(dth) * dis_xy;
        float dy = -sin(dth) *dis_xy;
        px +=(cos(theta) *dx - sin(theta) * dy);
        py +=(sin(theta) *dx + cos(theta) * dy);
    }

    if(dth != 0)
        theta += dth;
    last_left_encoder_count = robot_odom.odom->left_odom;
    last_right_encoder_count = robot_odom.odom->right_odom;
}

void cmd_vel_set(const geometry_msgs::Twist::ConstPtr& cmd_vel)
{
    WR_RobotPara wrRobotPara = {0xfe,0x07,0x04,0x00,0x00,0x00,0xff};

    unsigned char left_speed_set = 0;
    unsigned char right_speed_set = 0;

    SerialControl serialControl_speed;
    float left_speed_cmd = 0.0;
    float right_speed_cmd = 0.0;
    float cmd_x = cmd_vel->linear.x;
    float cmd_theta = cmd_vel->angular.z;
    //根据线速度和角速度计算左右轮子转速，弧度/秒
    left_speed_cmd  = (cmd_x - cmd_theta * WHEEL_SEPARATION / 2.0)/WHEEL_RADIUS;
    right_speed_cmd = (cmd_x + cmd_theta * WHEEL_SEPARATION / 2.0)/WHEEL_RADIUS;

    left_speed_cmd = 0.0 - left_speed_cmd;

    if (left_speed_cmd >0)
    {
        if (2 <= left_speed_cmd && left_speed_cmd < 6.8)
        {
            left_speed_cmd = 6.8;
        }
        else if (left_speed_cmd < 2)
        {
            left_speed_cmd = 0;
        }
    }

    if (left_speed_cmd < 0)
    {
        if (-2 >= left_speed_cmd && left_speed_cmd > -6.8)
        {
            left_speed_cmd = -6.8;
        }
        else if (left_speed_cmd > -2)
        {
            left_speed_cmd = 0;
        }
    }

    if (right_speed_cmd > 0)
    {
        if (2 <= right_speed_cmd && right_speed_cmd < 6.8)
        {
            right_speed_cmd = 6.8;
        }
        else if(right_speed_cmd < 2)
        {
            right_speed_cmd = 0;
        }
    }

    if (right_speed_cmd < 0)
    {
        if (-2 >= right_speed_cmd && right_speed_cmd > -6.8)
        {
            right_speed_cmd = -6.8;
        }
        else if(right_speed_cmd > -2)
        {
            right_speed_cmd = 0;
        }
    }


    printf("--------------------speed left is%f ------------------\n", left_speed_cmd);
    printf("speed right is%f\n", right_speed_cmd);

    if (left_speed_cmd >= 6.8)
    {
        if (left_speed_cmd > 16.4)
        {
            left_speed_cmd = 16.4;
        }
        left_speed_set = 27+(left_speed_cmd - 6.8)/0.1;
    }

    if (right_speed_cmd >= 6.8)
    {
        if (right_speed_cmd > 16.4)
        {
            right_speed_cmd = 16.4;
        }
        right_speed_set = 27+(right_speed_cmd - 6.8)/0.1;
    }

    if(left_speed_cmd <= -6.8)
    {
        if (left_speed_cmd < -16.4)
        {
            left_speed_cmd = -16.4;
        }
        left_speed_set = 19+(6.8 -left_speed_cmd)/0.1;

        left_speed_set = 0x80|left_speed_set;
    }

    if(right_speed_cmd <= -6.8)
    {
        if (right_speed_cmd < -16.4)
        {
            right_speed_cmd = -16.4;
        }

        right_speed_set = 19+(6.8 - right_speed_cmd)/0.1;

        right_speed_set = 0x80|right_speed_set;
    }

    printf("--------------------speed left is%x ------------------\n", left_speed_set);
    printf("speed right is%x\n", right_speed_set);

    wrRobotPara.ctl_flag2 = left_speed_set;
    wrRobotPara.ctl_flag3 = right_speed_set;

//    printf("serial speed set is%x", wrRobotPara);

    serialControl_speed.write_port(serialControl_speed.serial_fd,(const char *)&wrRobotPara,7);

}