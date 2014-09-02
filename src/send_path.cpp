#include <iostream>
#include <cmath>
#include <array>
#include <string>
#include <algorithm>
#include <fstream>

#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <nav_msgs/Odometry.h>

#include "turtlebot_fm/pathFM.h"
#include "turtlebot_fm/pathTurtlebot.h"
#include "turtlebot_fm/InitAngle.h"

#define PI 4*atan(1)

double x = 0, y = 0; // Position of the robot in X and Y axis
double x_ang, y_ang, z_ang, w_ang; // Rotation of the robot in quaternion form
double alpha; // Yaw rotation in rads
double init_angle = 0; // Initial angle

turtlebot_fm::pathFM pathFM; // Path from FM planner
bool enable_path=false;
bool enable_loop=true;
bool enable_rot=false;

// Obtain path from FM planner
void chatterCallback_path(const turtlebot_fm::pathFM::ConstPtr &msg)
{
    enable_path = msg->enable;
    pathFM.positions = msg -> positions;
    pathFM.vel_rate = msg -> vel_rate;
}

// Obtain initial rotation
void chatterCallback_rot(const turtlebot_fm::InitAngle::ConstPtr &msg)
{
    enable_rot = msg->enable;
    init_angle = msg->angle;
}

// Check if the robot have been finished its move
void chatterCallback_enable(const std_msgs::Bool::ConstPtr &msg)
{
    enable_loop=msg->data;
}

// Obtain measures from the odometry
void chatterCallback_odom(const nav_msgs::Odometry::ConstPtr &msg)
{
    x=msg->pose.pose.position.x;
    y=msg->pose.pose.position.y;
    x_ang=msg->pose.pose.orientation.x;
    y_ang=msg->pose.pose.orientation.y;
    z_ang=msg->pose.pose.orientation.z;
    w_ang=msg->pose.pose.orientation.w;
    // Calculate Yaw angle
    alpha = std::atan2(2*(x_ang*y_ang+w_ang*z_ang), w_ang*w_ang + x_ang*x_ang - y_ang*y_ang - z_ang*z_ang);

    if (alpha < 0)
        alpha = alpha + 2*PI;
}

int main(int argc, char **argv)
{

    ros::init(argc, argv, "FMPathServer");

    ros::NodeHandle n;

    ros::Subscriber sub_path = n.subscribe("path_FM", 1000, chatterCallback_path);
    ros::Subscriber sub_enable = n.subscribe("move_complete", 1000, chatterCallback_enable);
    ros::Subscriber sub_odom = n.subscribe("odom", 1000, chatterCallback_odom);
    ros::Subscriber sub_rot = n.subscribe("Init_Rotation", 1000, chatterCallback_rot);

    ros::Publisher chatter_path = n.advertise<turtlebot_fm::pathTurtlebot>("move_data", 1000);

    ros::Rate loop_rate(10);

    int init=1;
    int goal;
    int interval = 10;

    std::ofstream ofs;
    ofs.open ("file1.txt",  std::ofstream::out | std::ofstream::trunc);
    ofs.close();

    while (ros::ok())
    {
        turtlebot_fm::pathTurtlebot path_turtlebot;

        if (enable_path && enable_rot)
        {
            if (init<(int)pathFM.positions.size())
            {
                if (enable_loop)
                {
                    ROS_INFO("Path prepared");
                    double x0, x1, y0, y1, vel_rate;

		            double euc_dist= 0;
                    if (init+interval < (int)pathFM.positions.size())
                    {
                        goal = init+interval;

                        x0=pathFM.positions[init].dims[0];
                        x1=pathFM.positions[goal].dims[0];

                        y0=pathFM.positions[init].dims[1];
                        y1=pathFM.positions[goal].dims[1];

                        vel_rate = pathFM.vel_rate[init];

            			euc_dist=std::sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));

            			while (euc_dist < 0.1 && goal < (int)pathFM.positions.size())
                        {
                            goal++;
                            x1=pathFM.positions[goal].dims[0];
                            y1=pathFM.positions[goal].dims[1];

                            euc_dist=std::sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));
                        }

                    }
                    else
                    {
                        goal = (int)pathFM.positions.size()-1;
                        x0=pathFM.positions[init].dims[0];
                        x1=pathFM.positions[goal].dims[0];

                        y0=pathFM.positions[init].dims[1];
                        y1=pathFM.positions[goal].dims[1];

                        vel_rate = pathFM.vel_rate[init];

			            euc_dist=std::sqrt((x1-x0)*(x1-x0)+(y1-y0)*(y1-y0));

                        goal++;
                    }

                    double angle_aux=std::atan2((y1-y0), (x1-x0));

                    if (angle_aux<0)
                        angle_aux=angle_aux+2*PI;

                    double angle = angle_aux - (alpha + init_angle);

                    if (angle > PI)
                        angle = angle -2*PI;

                    if (angle < 0 && std::abs(angle) > PI)
                        angle = angle + 2*PI;



                    ofs.open ("file1.txt",  std::ios::out | std::ios::app);

                    ofs << x0-pathFM.positions[1].dims[0] << "\t" << y0-pathFM.positions[1].dims[1] << "\t" << x1-pathFM.positions[1].dims[0] << "\t" << y1-pathFM.positions[1].dims[1] << "\t" << x << "\t" << y << "\t" << angle << "\t" << euc_dist << std::endl;

                    ofs.close();

                    path_turtlebot.forward_distance = euc_dist;
                    path_turtlebot.turn_distance = angle ;
                    path_turtlebot.enable = true;
                    path_turtlebot.velocity_rate = vel_rate/3;

                    chatter_path.publish(path_turtlebot);

                    init = goal;
                    enable_loop = 0;

                    ROS_INFO("Path data sent");
                }
              }

        else
            enable_path=false;
        }

        ros::spinOnce();

        loop_rate.sleep();
    }



  return 0;
}
