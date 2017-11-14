//
// Created by chke on 7/14/17.
//

#include <iostream>
#include "../include/serial_control.h"

int serial_data_index = 0;
ROBOT_PARA robot_odom = {" ", 0, (struct ODOM*)malloc(sizeof(ODOM))};
RobotState_Info WaterSeat = {" ", 0x00};
RobotState_Info DoorState = {" ", 0x00};

SerialControl::SerialControl()
{
    serial_fd = 3;
    s_length = 0;
}

SerialControl::~SerialControl()
{

}

void * read_encoder_fun(void * para)
{
    SerialControl serialControl;
    SerialControl *pMotoControl = (SerialControl *)para;
    while (1) {
        pMotoControl ->read_serial_data();
    }
}

int SerialControl::open_serial_port(int index)
{
    serial_fd = open_port(index);
    if (serial_fd < 0){
        printf("Open channel encoder error");
        return -1;
    }
    else{
        printf("Open channel encoder success!\n");

        if(set_port(serial_fd,115200,8,'N',1)<0)
        {
            perror("set_opt error");
            return -1;
        }
    }
    tcflush(serial_fd, TCIFLUSH);
    return 0;
}

int SerialControl::close_serial_port()
{

    close_port(serial_fd);
    return 0;
}

int SerialControl::read_serial_data()
{

    do
    {
        if (read_port(serial_fd,serial_data_r,1) > 0)
        {
//            printf("serial data is %x\n\n", serial_data_r[0]);
            serial_data[serial_data_index] = serial_data_r[0];
            serial_data_index++;
        }
        else{
            if (0 <= (write_serial_data())){
                printf("read data error");
                return -1;
            }
            else{
                printf("prot is miss\n");
                close_serial_port();
                while ( -1 == open_serial_port(1))
                    sleep(1);
            }
        }

    }while (0xff != serial_data_r[0]);
    s_length = serial_data_index;
    serial_data_index = 0;
    serial_data_calssify();
    return 0;
}

int SerialControl::read_serial_task()
{
    int pid,ppid;
    if (pthread_create(&read_serial_threadid, NULL, read_encoder_fun, this) != 0)
    {
        printf("creat read_serial_task failed");
        close_port(serial_fd);
        return -1;
    }
    printf("creat read_serial_task success\n");
    pid = getpid();
    ppid = getppid();
    printf("%d,%d\n",pid,ppid);
    return 0;
}

int SerialControl::write_serial_data()
{
    if (write_port(serial_fd, "ifconnect",11) < 0) {
        return -1;
    }
    return 0;
}

int SerialControl::serial_data_calssify()
{
    int left_odom_plus;
    int right_odom_plus;

    unsigned char left_odom_plus_arry[10];
    unsigned char right_odom_plus_arry[10];

    for (int i = 0; i < s_length; i++) {
        printf("read data %d is %x\n", i, serial_data[i]);
        left_odom_plus_arry[i] = serial_data[i];
        right_odom_plus_arry[i] = serial_data[i];
    }
    printf("length %d\n", s_length);
    if (8 == s_length)
    {
        robot_odom.device_name = "odom";

        // left moto
        if(0 == (left_odom_plus_arry[3]&0x01))            //positive
        {
            left_odom_plus = (((left_odom_plus_arry[3]&0xe0)>>5)<<8)|left_odom_plus_arry[4];
            printf("left_odom_puls%d\n",left_odom_plus);
        }

        else if(1 == (left_odom_plus_arry[3]&0x01))            //nagetive
        {
            left_odom_plus = -((((left_odom_plus_arry[3]&0xe0)>>5)<<8)|left_odom_plus_arry[4]);
            printf("left_odom_puls%d\n",left_odom_plus);
        }
            //right moto
       if(0 == (right_odom_plus_arry[3] & 0x02))            //positive
        {
            right_odom_plus = -(((right_odom_plus_arry[3]&0x1c)>>2)<<8)|right_odom_plus_arry[5];
            printf("right_odom_puls%d\n",right_odom_plus);
        }
        else if(2 == (right_odom_plus_arry[3] & 0x02))            //nagetive
        {
            right_odom_plus = ((((right_odom_plus_arry[3]&0x1c)>>2)<<8)|right_odom_plus_arry[5]);
            printf("right_odom_puls%d\n",right_odom_plus);
        }
    }

    if(6 == s_length)
    {
        if ( 0xfc == serial_data[0])
        {
//            switch (serial_data[2])
//            {
//                case 0x01:
//                    WaterSeat.device_name = "water_seat";
//                    break;
//                case 0x03:
//                    DoorState.device_name = "door_state";
//                    break;
//                default:
//                    WaterSeat.device_name = "errorinfo";
//                    DoorState.device_name = "errorinfo";
//                    break;
//            }
            if (0x01 == serial_data[2])
            {
                WaterSeat.device_name = "water_seat";
                WaterSeat.info_byte = serial_data[3];
            }
            if (0x03 == serial_data[2])
            {
                DoorState.device_name = "door_state";
                DoorState.info_byte = serial_data[3];
            }
        }
    }

    robot_odom.odom->left_odom += left_odom_plus;
    robot_odom.odom->right_odom += right_odom_plus;
}


