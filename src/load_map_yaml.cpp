#include <iostream>
#include <cmath>
#include <array>
#include <string>
#include <algorithm>
#include <fstream>

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/MapMetaData.h>

#include "turtlebot_fm/map.h"

#include <sstream>

nav_msgs::OccupancyGrid dataOccupancyROS; // occupancy grid of the map
std::vector <int> size; // size of the map
int ndims; // number of dimensions of the map
float resolution; // size of cells of the grid

int width, height;

bool enable_map=false;
bool enable_metadata=false;

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
    ros::Subscriber subscriber_map = n.subscribe("map", 1000, chatterCallback_map);
    ros::Subscriber subscriber_metadata = n.subscribe("map_metadata", 1000, chatterCallback_metadata);

    ros::Rate loop_rate(10);

    while (ros::ok())
    {
        turtlebot_fm::map map;

        // Only works when all the information have been recieved
        if (enable_map==true && enable_metadata==true)
        {
            // Save data
            map.gridSize.push_back(width);
            map.gridSize.push_back(height);
            map.resolution = resolution;
            map.ndims = 2;

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
            }

            // Disable the loop and wait to a new map
            enable_map=false;
            enable_metadata=false;

            ROS_INFO("Map loaded");

            // Publish the map
            mapFM.publish(map);

            ROS_INFO("Map published");
        }

        ros::spinOnce();

        loop_rate.sleep();
    }

  return 0;
}
