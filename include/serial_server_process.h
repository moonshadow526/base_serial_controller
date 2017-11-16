//
// Created by chke on 11/10/17.
//

#ifndef PROJECT_SERIAL_SERVER_PROCESS_H
#define PROJECT_SERIAL_SERVER_PROCESS_H

extern pthread_rwlock_t rwlock;
extern struct ROBOTSTATE_INFO WaterSeat;
extern struct ROBOTSTATE_INFO DoorState;

bool serial_server_process(serila_controller::serial_data_interactive::Request &req, serila_controller::serial_data_interactive::Response &res);

#endif //PROJECT_SERIAL_SERVER_PROCESS_H
