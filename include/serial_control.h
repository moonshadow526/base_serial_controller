//
// Created by chke on 7/14/17.
//

#ifndef PROJECT_READ_ENCODER_H
#define PROJECT_READ_ENCODER_H

#include "serial.h"
#include <sys/socket.h>

//extern int hotplug_sock;
//#define UEVENT_BUFFER_SIZE 2048

class SerialControl : public  Serial
{
public:
    SerialControl();
    ~SerialControl();
    int open_serial_port(int index);
    int close_serial_port();
    int read_serial_data();
    int read_serial_task();
    int write_serial_data();
    int serial_data_calssify();
    int serial_fd;
private:
    pthread_t read_serial_threadid;
    unsigned char serial_data[10];
    unsigned char serial_data_r[10];
    int s_length;

};

typedef struct ODOM
{
   int left_odom;
   int right_odom;
}Odom;

typedef struct ROBOTSTATE_INFO
{
    std::string device_name;
    unsigned char info_byte[2];
    bool pub_flag;
}RobotState_Info;

typedef struct Robot_Para
{
    std::string device_name;
    int state;
    Odom *odom;
}ROBOT_PARA;


typedef struct WR_SPEEDPARA
{
    unsigned char  head;
    unsigned char length;
    unsigned char ctl_flag1;
    unsigned char ctl_flag2;
    unsigned char ctl_flag3;
    unsigned char chk_sum;
    unsigned char tail;
}WR_SpeedPara;

typedef struct WR_ROBOTSTATE
{
    unsigned char  head;
    unsigned char length;
    unsigned char ctl_flag1;
    unsigned char ctl_flag2;
    unsigned char chk_sum;
    unsigned char tail;
}WR_RobotState;

extern RobotState_Info WaterSeat;
extern RobotState_Info DoorState;
extern unsigned char wr_flag;

#endif //PROJECT_READ_ENCODER_H
