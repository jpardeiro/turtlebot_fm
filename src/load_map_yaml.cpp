#include <iostream>
#include <cmath>
#include <array>
#include <string>
#include <algorithm>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf/transform_broadcaster.h>
#include <gazebo/gazebo.hh>
#include "gazebo_msgs/ModelState.h"
#include "gazebo_msgs/ModelStates.h"
#include <gazebo_msgs/SetModelState.h>

#include "../include/turtlebot_fm/io/gridpoints.hpp"
#include "../include/turtlebot_fm/fmdata/fmcell.h"
#include "../include/turtlebot_fm/ndgridmap/ndgridmap.hpp"

#include "turtlebot_fm/map.h"
#include "turtlebot_fm/InitAndGoal.h"
#include "turtlebot_fm/InitAngle.h"

#include <sstream>

nav_msgs::OccupancyGrid dataOccupancyROS; // occupancy grid of the map
std::vector <int> size; // size of the map
int ndims; // number of dimensions of the map
float resolution; // size of cells of the grid

int width, height;

bool enable_map = false;
bool enable_metadata = false;
bool enable_broadcaster = false;

// Recieve map occupancy Grid from map-server
void chatterCallback_map(const nav_msgs::OccupancyGrid::ConstPtr &msg)
{
    ROS_INFO("Map recieved");
    dataOccupancyROS.data=msg->data;
    enable_map=true;
}

// Recieve map data from map-server
void chatterCallback_metadata(const nav_msgs::MapMetaData::ConstPtr &msg)
{
    ROS_INFO("Map metadata recieved");
    width=msg->width;
    height=msg->height;
    resolution=msg->resolution;
    enable_metadata=true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "load_map_yaml");

    ros::NodeHandle n;

    // node publishers and subscribers
    ros::Publisher mapFM = n.advertise<turtlebot_fm::map>("map_FM", 1000);
    ros::Publisher Init_And_Goal = n.advertise<turtlebot_fm::InitAndGoal>("Init_and_Goal_Points", 1000);
    ros::Publisher Init_Rotation = n.advertise<turtlebot_fm::InitAngle>("Init_Rotation", 1000);
    //ros::Publisher pub_ = n.advertise<geometry_msgs::PoseWithCovarianceStamped>("initialpose", 1000);
    ros::Subscriber subscriber_map = n.subscribe("map", 1000, chatterCallback_map);
    ros::Subscriber subscriber_metadata = n.subscribe("map_metadata", 1000, chatterCallback_metadata);

    ros::Rate loop_rate(10);

    constexpr int ndims_ = 2;
    std::array<int, ndims_> coords_init, coords_goal;

    static tf::TransformBroadcaster br;
    tf::Transform transform;

    while (ros::ok())
    {
        turtlebot_fm::map map;
        turtlebot_fm::InitAndGoal initandgoal;
        turtlebot_fm::InitAngle initangle;

        

        

        // Only works when all the information have been recieved
        if (enable_map==true && enable_metadata==true)
        {
            // Save data
            map.gridSize.push_back(width);
            map.gridSize.push_back(height);
            map.resolution = resolution;
            map.ndims = 2;

            
            nDGridMap<FMCell, ndims_> grid;

            std::array<int, ndims_> dimsize = {width, height};

            grid.resize(dimsize);

            grid.setLeafSize(resolution);

            /* Convert the map grid information. Fast Marching works with
               occupancy grids and all cells of the grid have to be occupied
               or not. In unknown cases the cell is valuated as occupied*/
            for (int i = 0; i < width*height; i++)
            {
                int occupancy = (int)dataOccupancyROS.data[i];

                bool occ;

                // Free cells
                if (occupancy >= 50)
                    occ = 0;
                // Occupied cells
                else if (occupancy < 50 && occupancy >= 0)
                    occ = 1;
                // Unknown cells
                else if (occupancy < 0)
                    occ = 0;
                map.occupancyGrid.push_back(occ);
                grid.getCell(i).setOccupancy(occ);
            }

            double init_angle = 0;

            std::cout << "Set initial rotation of the robot (in rads): " << std::endl;
            std::cin >> init_angle;

            
            GridPoints::selectMapPoints(grid, coords_init, coords_goal);

            int idx, goal;
            grid.coord2idx(coords_init, idx);
            grid.coord2idx(coords_goal, goal);

            initandgoal.enable = true;
            initandgoal.init = idx;
            initandgoal.goal = goal;

            ROS_INFO("Initial and Goal Points selected");

            Init_And_Goal.publish(initandgoal);

            ROS_INFO("Initial and Goal Points published");

            initangle.enable = true;
            initangle.angle = init_angle;

            ROS_INFO("Initial Rotation selected");

            Init_Rotation.publish(initangle);

            ROS_INFO("Initial Rotation published");

            geometry_msgs::Pose start_pose;
            start_pose.position.x = coords_init[0]*resolution;
            start_pose.position.y = coords_init[1]*resolution;
            start_pose.position.z = 0.1;
            start_pose.orientation.x = 0.0;
            start_pose.orientation.y = 0.0;
            start_pose.orientation.z = 0.0;
            start_pose.orientation.w = 0.0;

            geometry_msgs::Twist start_twist;
            start_twist.linear.x = 0.0;
            start_twist.linear.y = 0.0;
            start_twist.linear.z = 0.0;
            start_twist.angular.x = 0.0;
            start_twist.angular.y = 0.0;
            start_twist.angular.z = init_angle;

            gazebo_msgs::ModelState modelstate;
            modelstate.model_name = (std::string) "mobile_base";
            modelstate.pose = start_pose;
            modelstate.twist = start_twist;

            ros::ServiceClient client = n.serviceClient<gazebo_msgs::SetModelState>("/gazebo/set_model_state");
            gazebo_msgs::SetModelState setmodelstate;
            setmodelstate.request.model_state = modelstate;
            client.call(setmodelstate);

            transform.setOrigin( tf::Vector3((coords_init[0]+1)*resolution, (coords_init[1]+1)*resolution, 0.0) );
            tf::Quaternion q;
            q.setRPY(0, 0, init_angle);
            transform.setRotation(q);

            // Disable the loop and wait to a new map
            enable_map = false;
            enable_metadata = false;
            enable_broadcaster = true;

            ROS_INFO("Map loaded");

            // Publish the map
            mapFM.publish(map);

            ROS_INFO("Map published");
        }

        if (enable_broadcaster)
            while (ros::ok())
                br.sendTransform(tf::StampedTransform(transform, ros::Time::now(), "/map", "/odom"));

        ros::spinOnce();

        loop_rate.sleep();
    }

  return 0;
}
