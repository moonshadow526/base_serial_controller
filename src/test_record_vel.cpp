//
// Created by chke on 11/24/17.
//

#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <fstream>
#include <iostream>
#include <pthread.h>


pthread_rwlock_t rwlock_w =PTHREAD_RWLOCK_INITIALIZER;



void recv_cmd_vel(const geometry_msgs::Twist::ConstPtr& msg){
    pthread_rwlock_wrlock(&rwlock_w);
    std::ofstream out("/home/robot/record_vel.txt",std::ios::app);
    if(out.is_open()){
        out<<"x:"<<msg->linear.x<<"    z:"<<msg->angular.z<<"\n";
        out.close();
    }else{
        std::cout<<"no such file"<<std::endl;
    }
    pthread_rwlock_unlock(&rwlock_w);
}


int main(int argv,char** argc){
    std::fstream file("/home/robot/record_vel.txt",std::ios::out);
    file.close();

    ros::init(argv,argc,"test_record_vel");
    ros::NodeHandle n;
    ros::Subscriber cmd_sub = n.subscribe<geometry_msgs::Twist>("/cmd_vel", 10, recv_cmd_vel);

    ros::spin();


    return 0;

}