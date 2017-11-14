//
// Created by chke on 7/14/17.
//

#ifndef PROJECT_SERIAL_H
#define PROJECT_SERIAL_H
#include  <stdio.h>
#include  <stdlib.h>
#include  <unistd.h>
#include  <sys/types.h>
#include  <sys/signal.h>
#include  <sys/stat.h>
#include  <fcntl.h>
#include  <termios.h>
#include  <errno.h>
#include  <limits.h>
#include  <string.h>
#include <string>

class Serial
{
public:
    Serial();
    ~Serial();
    int open_port(int index);
    void close_port(int fd);
    int set_port(int fd, int nSpeed, int nBits, char nEvent, int nStop);
    int write_port(int fd,const char *data,int datalength);
    int read_port(int fd,unsigned char *data,int datalength);
};



#endif //PROJECT_SERIAL_H
