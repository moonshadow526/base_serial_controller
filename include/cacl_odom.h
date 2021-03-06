//
// Created by chke on 11/11/17.
//

#ifndef PROJECT_CACL_ODOM_H
#define PROJECT_CACL_ODOM_H

#include <geometry_msgs/Twist.h>

//#define speedlimt

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



extern pthread_rwlock_t rwlock;
extern bool wr_zero_speed;
extern int nfd[5];

#endif //PROJECT_CACL_ODOM_H
