//
// Created by chke on 7/14/17.
//

#include <iostream>
#include "../include/serial_control.h"
#include "../include/debug.h"

int odom_serial_data_index = 0;
int imu_serial_data_index = 0;
bool odom_length_chk = true;
bool imu_length_chk = true;
//bool pub_status = true;
double imu_yaw;

ROBOT_PARA robot_odom = {" ", 0, (struct ODOM*)malloc(sizeof(ODOM))};
RobotState_Info WaterSeat = {" ", {0x00,0x00}, false};
RobotState_Info DoorState = {" ", {0x00,0x00}, false};
unsigned char last_wr_flag = 0x01;

SerialControl::SerialControl()
{
    serial_fd = -1;
    s_length = 0;
}

SerialControl::~SerialControl()
{

}

void * read_encoder_fun(void * para)
{
    int i = *(int *)para;
    SerialControl *pMotoControl = (SerialControl *)para;
    while (1) {
        pMotoControl ->read_serial_data();
    }
}

int SerialControl::open_serial_port(int index)
{
    if (1 == index)
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
    }

    else if (2 == index)
    {
        imu_fd = open_port(index);
        if (imu_fd < 0){
            printf("Open channel encoder error");
            return -1;
        }
        else{
            printf("Open channel imu_serial success!\n");

            if(set_port(imu_fd,115200,8,'N',1)<0)
            {
                perror("set_opt error");
                return -1;
            }
        }
        tcflush(imu_fd, TCIFLUSH);
    }

    return 0;
}

int SerialControl::close_serial_port()
{

    close_port(serial_fd);
    return 0;
}

int SerialControl::read_serial_data()
{

    odom_length_chk = true;
    do
    {
        if (read_port(encord_serial,serial_data_r,1) > 0)
        {
//            printf("serial data is %x\n", serial_data_r[0]);
            serial_data[odom_serial_data_index] = serial_data_r[0];
            odom_serial_data_index++;
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

        if (odom_serial_data_index > 9)
        {
            odom_length_chk = false;
            break;
        }

    }while (0xff != serial_data_r[0]);
    s_length = odom_serial_data_index;
    odom_serial_data_index = 0;
    serial_data_calssify();
    return 0;
}

int SerialControl::read_imu_data()
{

    imu_length_chk = true;
//    do
//    {
//        if (read_port(fd,serial_data_r,1) > 0)
//        {
////            printf("serial data is %x\n\n", serial_data_r[0]);
//            serial_data[imu_serial_data_index] = serial_data_r[0];
//            imu_serial_data_index++;
//        }
//        else{
//            if (0 <= (write_serial_data())){
//                printf("read data error");
//                return -1;
//            }
//            else{
//                printf("prot is miss\n");
//                close_serial_port();
//                while ( -1 == open_serial_port(1))
//                    sleep(1);
//            }
//        }
//
//        if (imu_serial_data_index > 19)
//        {
//            imu_length_chk = false;
//            break;
//        }
//
//    }while (0xBB != serial_data_r[0]&&imu_serial_data_index != 12);
//    printf("imu_fd is %d\n")
    if (read_port(imu_serial,imu_serial_data_r,1) > 0 && imu_serial_data_r[0] == 0x7a)
    {
        if (read_port(imu_serial,imu_serial_data_r,1) > 0 && imu_serial_data_r[0] == 0x7b)
        {
            if (read_port(imu_serial,imu_serial_data_r,1) > 0 && imu_serial_data_r[0] == 0x12)
            {
                for (int i = 0; i < 17 ; i++)
                {
                    read_port(imu_serial,imu_serial_data_r,1);
                    imu_serial_data[i] = imu_serial_data_r[0];
                    printf("imu_serial_data %d is %x\n", i, imu_serial_data_r[0]);
                }
            }
        }
    }

//    s_length = imu_serial_data_index;
//    imu_serial_data_index = 0;
    imu_data_calssify();
    return 0;
}

int SerialControl::read_serial_task()
{
    int pid,ppid;
    if (pthread_create(&read_serial_threadid, NULL, read_encoder_fun, &serial_fd) != 0)
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
        DEBUG("read data %d is %x\n", i, serial_data[i]);
        left_odom_plus_arry[i] = serial_data[i];
        right_odom_plus_arry[i] = serial_data[i];
    }
    DEBUG("length %d\n", s_length);
    if (8 == s_length && odom_length_chk == true)
    {
        robot_odom.device_name = "odom";

        // left moto
        if(0 == (left_odom_plus_arry[3]&0x01))            //positive
        {
            left_odom_plus = (((left_odom_plus_arry[3]&0xe0)>>5)<<8)|left_odom_plus_arry[4];
            left_odom_plus = -left_odom_plus;
            DEBUG("left_odom_puls%d\n",left_odom_plus);
        }

        else if(1 == (left_odom_plus_arry[3]&0x01))            //nagetive
        {
            left_odom_plus = -((((left_odom_plus_arry[3]&0xe0)>>5)<<8)|left_odom_plus_arry[4]);
            left_odom_plus = -left_odom_plus;
            DEBUG("left_odom_puls%d\n",left_odom_plus);
        }
            //right moto
       if(0 == (right_odom_plus_arry[3] & 0x02))            //nagetive
        {
            right_odom_plus = -((((right_odom_plus_arry[3]&0x1c)>>2)<<8)|right_odom_plus_arry[5]);
            right_odom_plus = -right_odom_plus;
            DEBUG("right_odom_puls%d\n",right_odom_plus);
        }
        else if(2 == (right_odom_plus_arry[3] & 0x02))            //positive
        {
            right_odom_plus = ((((right_odom_plus_arry[3]&0x1c)>>2)<<8)|right_odom_plus_arry[5]);
            right_odom_plus = -right_odom_plus;
            DEBUG("right_odom_puls%d\n",right_odom_plus);
        }
    }

    if(6 == s_length && odom_length_chk == true)
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
                if(wr_flag == ~last_wr_flag ){
                    WaterSeat.info_byte[1] = serial_data[3];
                    WaterSeat.pub_flag = false;
                    last_wr_flag = wr_flag;
                }
                else if(!WaterSeat.pub_flag)
                {
                    WaterSeat.info_byte[0] = serial_data[3];
                    WaterSeat.pub_flag = true;
                }
                else
                    WaterSeat.pub_flag = false;

            }
            if (0x03 == serial_data[2])
            {
                DoorState.device_name = "door_state";
                if(wr_flag == ~last_wr_flag ){
                    DoorState.info_byte[1] = serial_data[3];
                    DoorState.pub_flag = false;
                    last_wr_flag = wr_flag;
                }
                else if(!DoorState.pub_flag)
                {
                    DoorState.info_byte[0] = serial_data[3];
                    DoorState.pub_flag = true;
                }
                else
                    DoorState.pub_flag = false;
            }
        }
    }

    robot_odom.odom->left_odom += left_odom_plus;
    robot_odom.odom->right_odom += right_odom_plus;
}

float SerialControl::imu_data_calssify()
{
    imu_yaw = (double)(imu_serial_data[1]<<16|imu_serial_data[2]<<8|imu_serial_data[3])*2*3.14159/3600000;
//    printf("yaw_data%f\n", yaw_data);
    return imu_yaw;
}
