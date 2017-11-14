//
// Created by chke on 11/11/17.
//

#ifndef PROJECT_CACL_ODOM_H
#define PROJECT_CACL_ODOM_H

#include <geometry_msgs/Twist.h>

//两轮子的间距-米
#define WHEEL_SEPARATION       0.296
//轮子半径-米
#define WHEEL_RADIUS           0.050
//轮子前进每米编码器产生的脉冲数
#define TICKSPERMETER  22918.3
//每弧度（轮子的，注意乘以减速比）编码器产生的脉冲数
#define TICKSPERRADIAN 1145.9

void calc_odom(void);
void cmd_vel_set(const geometry_msgs::Twist::ConstPtr& cmd_vel);

typedef struct WR_ROBOTPARA
{
    unsigned char  head;
    unsigned char length;
    unsigned char ctl_flag1;
    unsigned char ctl_flag2;
    unsigned char ctl_flag3;
    unsigned char chk_sum;
    unsigned char tail;
}WR_RobotPara;

#endif //PROJECT_CACL_ODOM_H
