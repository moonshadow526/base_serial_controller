//
// Created by chke on 11/10/17.
//

#include <serila_controller/serial_data_interactive.h>
#include "../include/serial_server_process.h"
#include "../include/serial_control.h"

unsigned char wr_flag = 0x01;

bool serial_server_process(serila_controller::serial_data_interactive::Request &req, serila_controller::serial_data_interactive::Response &res)
{
    WR_RobotState wrRobotState = {0xfe,0x06,0x00,0x00,0x00,0xff};

    SerialControl ser_serialControl;
    if(req.get_msg == " ")
    {
        wrRobotState.ctl_flag1 = 0x02;
        wrRobotState.ctl_flag2 = req.get_val;
        pthread_rwlock_wrlock(&rwlock);
        ser_serialControl.write_port(ser_serialControl.serial_fd, (const char *)&wrRobotState ,6);
        wr_flag = ~wr_flag;
        pthread_rwlock_unlock(&rwlock);

        res.echo_msg = " ";
        res.echo_val = WaterSeat.info_byte[1];
    }

    if(req.get_msg == " ")
    {
        wrRobotState.ctl_flag1 = 0x03;
        wrRobotState.ctl_flag2 = req.get_val;
        pthread_rwlock_wrlock(&rwlock);
        ser_serialControl.write_port(ser_serialControl.serial_fd, (const char *)&wrRobotState ,6);
        wr_flag = ~wr_flag;
        pthread_rwlock_unlock(&rwlock);

        res.echo_msg = " ";
        res.echo_val = DoorState.info_byte[1];

    }
    return true;
}