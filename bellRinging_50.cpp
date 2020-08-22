//
// Created by kjwang on 2020/2/19.
//
//本示例程序供大家学习probot_anno的位置及速度控制如何编程，展示了：
//1.如何使用ikfast求解器帮我们去计算机械臂逆解;
//2.如何创建位置控制数据发布者，并发布位置数据（速度控制也一样的，把名字换换而已），控制gazebo的机械臂动起来。
//



#include <string>
#include <ros/ros.h>
#include <iostream>
#include <time.h>
#include "vector"
#include "std_msgs/Float32.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "controller_manager_msgs/SwitchController.h"
#include "controller_manager_msgs/ListControllers.h"
#include "ikfast.h"
#include "probot_anno_manipulator_ikfast_moveit_plugin.cpp"
#include <eigen3/Eigen/Dense>

using namespace ikfast_kinematics_plugin;

double path_plan(double dis, double time_tot, double time_cur){
    double A = (6.0 * dis) / pow(time_tot, 5);
    double B = (-15.0 * dis) / pow(time_tot, 4);
    double C = (10 * dis) / pow(time_tot, 3);
    return (A * 5 * pow(time_cur, 4) + B * 4 * pow(time_cur, 3) + C * 3 * pow(time_cur, 2));
}

int main(int argc, char **argv) {

    bool ret;
    //节点初始化
    ros::init(argc, argv, "control_example");
    ros::NodeHandle node_handle;
    ros::AsyncSpinner spinner(1);
    ROS_INFO_STREAM("start");
    //double targetPose[6] = {0, -1.5708, 1.5708, 0, 1.5708, 0};//运动的起点位置
    //创建发布者对象，用于发布位置信息，
    //位置控制的话题名为："/probot_anno/arm_pos_controller/command"
    //速度控制的话题名为："/probot_anno/arm_vel_controller/command"
    //发送的数据类型均为：std_msgs::Float64MultiArray，
    //它的创建与赋值方法在下面有，一组6个浮点数
    //ros::Publisher pos_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_pos_controller/command", 100);
    ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float32MultiArray>("speed_chatter", 1000);
    // ros::Publisher vel_pub = node_handle.advertise<std_msgs::Float64MultiArray>("/probot_anno/arm_vel_controller/command", 1000);
    //初始化publish的变量并初始化6个值
    // std_msgs::Float64MultiArray init_vel;
    std_msgs::Float32MultiArray init_vel;
    init_vel.data.push_back(0.0);
    init_vel.data.push_back(0.0);
    init_vel.data.push_back(0.0);
    init_vel.data.push_back(0.0);
    init_vel.data.push_back(0.0);
    init_vel.data.push_back(0.0);
    sleep(1);
    
    //frequency = 50;
    double theta2 = 0, theta3 = 0, theta5 = 0;
    double v[301] = {}, omega1[301] = {}, omega2[301] = {}, omega3[301] = {}, omega5[301] = {};
    Eigen::Matrix3d Jacob;
    Jacob = Eigen::Matrix3d::Zero();
    Eigen::Vector3d Vcur;
    Vcur = Eigen::Vector3d::Zero();
    Eigen::Vector3d omega;
    omega = Eigen::Vector3d::Zero();
    for (int t = 1; t <= 300; ++t){
        v[t] = path_plan(0.3993030178, 6.0, t / 50.0);
        omega1[t] = path_plan(-0.7086262721, 6.0, t / 50.0);
    }
    for (int t = 1; t <= 300; ++t){
        Jacob << (-0.225 * cos(theta2) - 0.2289 * sin(theta2 + theta3) + 0.055 * cos(theta2 + theta3 + theta5)),
                 (-0.2289 * sin(theta2 + theta3) + 0.055 * cos(theta2 + theta3 + theta5)),
                 (0.055 * cos(theta2 + theta3 + theta5)),
                 (-0.225 * sin(theta2) + 0.2289 * cos(theta2 + theta3) + 0.055 * sin(theta2 + theta3 + theta5)),
                 (0.2289 * cos(theta2 + theta3) + 0.055 * sin(theta2 + theta3 + theta5)),
                 (0.055 * sin(theta2 + theta3 + theta5)),
                 1, 1, 1;
        Vcur << v[t] * 0.3503104003, v[t] * (-0.9366320396), 0;
    omega = Jacob.colPivHouseholderQr().solve(Vcur);
    omega2[t] = omega[0];
    omega3[t] = omega[1];
    omega5[t] = omega[2];
    theta2 = theta2 + omega2[t] * 0.02;
    theta3 = theta3 + omega3[t] * 0.02;
    theta5 = theta5 + omega5[t] * 0.02;
    }
    //for (int t = 1; t <= 200; ++t)
    //    cout<<v[t]<<"   "<<omega2[t]<<" "<<omega3[t]<<" "<<omega5[t]<<endl;
    //为要发送的变量装入解出的六关节坐标
    //如果是要发送速度，那也是类似于这样，装入6个关节角速度值（弧度制）即可
    ros::Rate loop_rate(50);
    for(int i = 1; i <= 300; ++i){
        init_vel.data.at(0) = omega1[i] * 30 * 180 / M_PI;
        init_vel.data.at(1) = omega2[i] * 205 * 180 / (3 * M_PI);
        init_vel.data.at(2) = omega3[i] * 50 * 180 / M_PI;
        init_vel.data.at(3) = 0;
        init_vel.data.at(4) = omega5[i] * 125 * 180 / (2 * M_PI);
        init_vel.data.at(5) = 0;
        //发送出去，若成功，机械臂状态会改变
        vel_pub.publish(init_vel);
        ROS_INFO_STREAM("published");
        loop_rate.sleep();
    }

    for (int t = 1; t <= 100; ++t){
        if(t <= 50) v[t] = path_plan(0.08587535937, 1.0, t / 50.0);
        else v[t] = path_plan(0.1279324184, 1.0, (t - 50) / 50.0);
        omega1[t] = path_plan(1.231280165, 2.0, t / 50.0) * 1.3;
        }
    for (int t = 1; t <= 100; ++t){
        Jacob << (-0.225 * cos(theta2) - 0.2289 * sin(theta2 + theta3) + 0.055 * cos(theta2 + theta3 + theta5)),
                 (-0.2289 * sin(theta2 + theta3) + 0.055 * cos(theta2 + theta3 + theta5)),
                 (0.055 * cos(theta2 + theta3 + theta5)),
                 (-0.225 * sin(theta2) + 0.2289 * cos(theta2 + theta3) + 0.055 * sin(theta2 + theta3 + theta5)),
                 (0.2289 * cos(theta2 + theta3) + 0.055 * sin(theta2 + theta3 + theta5)),
                 (0.055 * sin(theta2 + theta3 + theta5)),
                 1, 1, 1;
        if(t <= 50) Vcur << v[t] * 0.3635294446, v[t] * 0.93158271, 0;
        else Vcur << v[t] * (-0.7803602937), v[t] * (-0.6253301626), 0;
    omega = Jacob.colPivHouseholderQr().solve(Vcur);
    omega2[t] = omega[0];
    omega3[t] = omega[1];
    omega5[t] = omega[2];
    theta2 = theta2 + omega2[t] * 0.02;
    theta3 = theta3 + omega3[t] * 0.02;
    theta5 = theta5 + omega5[t] * 0.02;
    }

    for(int cnt = 1; ; ++cnt){
        //为要发送的变量装入解出的六关节坐标
        //如果是要发送速度，那也是类似于这样，装入6个关节角速度值（弧度制）即可
        for(int i = 1; i <= 100; ++i){
            init_vel.data.at(0) = omega1[i] * 30 * 180 / M_PI;
            init_vel.data.at(1) = omega2[i] * 205 * 180 / (3 * M_PI);
            init_vel.data.at(2) = omega3[i] * 50 * 180 / M_PI;
            init_vel.data.at(3) = 0;
            init_vel.data.at(4) = omega5[i] * 125 * 180 / (2 * M_PI);
            init_vel.data.at(5) = 0;
            //发送出去，若成功，机械臂状态会改变
            vel_pub.publish(init_vel);
            ROS_INFO_STREAM("published");
            loop_rate.sleep();
        }
        for(int i = 100; i >= 1; --i){
            init_vel.data.at(0) = -omega1[i] * 30 * 180 / M_PI;
            init_vel.data.at(1) = -omega2[i] * 205 * 180 / (3 * M_PI);
            init_vel.data.at(2) = -omega3[i] * 50 * 180 / M_PI;
            init_vel.data.at(3) = 0;
            init_vel.data.at(4) = -omega5[i] * 125 * 180 / (2 * M_PI);
            init_vel.data.at(5) = 0;
            //发送出去，若成功，机械臂状态会改变
            vel_pub.publish(init_vel);
            ROS_INFO_STREAM("published");
            loop_rate.sleep();
        }
    }
}
