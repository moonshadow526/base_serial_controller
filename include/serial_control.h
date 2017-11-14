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

//Odom odom_t = {0,0};

typedef struct Robot_Para
{
    std::string device_name;
    int state;
    Odom *odom;
}ROBOT_PARA;





#endif //PROJECT_READ_ENCODER_H
