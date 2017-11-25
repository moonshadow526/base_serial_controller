//
// Created by chke on 11/10/17.
//

#include <serila_controller/serial_data_interactive.h>
#include "../include/serial_server_process.h"
#include "../include/serial_control.h"
#include "../include/debug.h"

unsigned char wr_flag = 0x01;

bool serial_server_process(serila_controller::serial_data_interactive::Request &req, serila_controller::serial_data_interactive::Response &res)
{
    WR_RobotState wrRobotState = {0xfe,0x06,0x00,0x00,0x00,0xff};

    SerialControl ser_serialControl;
    int wr_len;

    if(req.get_msg == "choose_grid")
    {
        wrRobotState.ctl_flag1 = 0x01;
        wrRobotState.ctl_flag2 = req.get_val-1;
        printf("choose grid data is %x\n",wrRobotState.ctl_flag2);
        pthread_rwlock_wrlock(&rwlock);
        ser_serialControl.write_port(nfd[encord_serial], (const char *)&wrRobotState ,6);
        wr_flag = ~wr_flag;
        pthread_rwlock_unlock(&rwlock);

        res.echo_msg = "choose_grid";
        res.echo_val = WaterSeat.info_byte[1];
    }

    if(req.get_msg == "light_grid")
    {
        wrRobotState.ctl_flag1 = 0x02;
        wrRobotState.ctl_flag2 = req.get_val-1;
        printf("light grid data is %x\n",wrRobotState.ctl_flag2);
        pthread_rwlock_wrlock(&rwlock);
        wr_len = ser_serialControl.write_port(nfd[encord_serial], (const char *)&wrRobotState ,6);
        wr_flag = ~wr_flag;
        pthread_rwlock_unlock(&rwlock);

        if (6 == wr_len)
        {
            res.echo_msg = "light_grid";
            res.echo_val = WaterSeat.info_byte[1];
        }
        else
        {
            res.echo_msg = "error";
            res.echo_val = WaterSeat.info_byte[1];
        }

    }

    if(req.get_msg == "door")
    {
        wrRobotState.ctl_flag1 = 0x03;
        wrRobotState.ctl_flag2 = req.get_val;
        printf("door grid data is %x\n",wrRobotState.ctl_flag2);
        pthread_rwlock_wrlock(&rwlock);
        ser_serialControl.write_port(nfd[encord_serial], (const char *)&wrRobotState ,6);
        wr_flag = ~wr_flag;
        pthread_rwlock_unlock(&rwlock);

        if (6 == wr_len)
        {
            res.echo_msg = "door";
            res.echo_val = DoorState.info_byte[1];
        }
        else
        {
            res.echo_msg = "error";
            res.echo_val = DoorState.info_byte[1];
        }

    }
    return true;
}