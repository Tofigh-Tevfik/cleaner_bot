#include "coverage_path_planner.h"
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <geometry_msgs/Pose.h>
#include <nav_msgs/Odometry.h>
#include <tf/tf.h>

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
geometry_msgs::Pose calculatePose(int, int, int, int, float);
void odom_callback(const nav_msgs::Odometry::ConstPtr&);
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr&);

// variable for the map
nav_msgs::OccupancyGrid occupancy_grid_map;
bool updated = false;
// robot position
geometry_msgs::Pose robot_pose;
// grid lookahead
float mu = 0.5;
int origin[2] = {25, 25};

int main(int argc, char **argv)
{
    // initialize ros node
    ros::init(argc, argv, "converage_path_planner");
    ros::NodeHandle node;

    // subscribing to /map topic to get map data
    ros::Subscriber map_sub = node.subscribe<nav_msgs::OccupancyGrid>("/map", 1, map_callback);
    ros::Subscriber odom_sub = node.subscribe<nav_msgs::Odometry>("/odom", 100, odom_callback);
    ros::Rate sleep_rate(10);

    // waiting until the map is recieved over the callback
    while (ros::ok()) {
        ros::spinOnce();
        if (updated) {
            break;
        }
        sleep_rate.sleep();
    }

    unsigned int height = occupancy_grid_map.info.height;
    unsigned int width = occupancy_grid_map.info.width;
    std::cout << "height: " << height << std::endl; 
    std::cout << "width: " << width << std::endl;

    float real_resolution = 0.05; // resolution of the map which is published in /map topic
    float desired_resolution = 0.4; // desired resolution to perform the coverage path planning
    int grid_ratio = desired_resolution / real_resolution;

    unsigned int reduced_height = height / grid_ratio;
    unsigned int reduced_width = width / grid_ratio;

    int** reshaped_map = reshape(occupancy_grid_map.data.data(), height, width); // reshaping the map
    plot_org_map(reshaped_map, height, width); // plotting the map

    // reducing the resolution of the map before planning the coverage path
    int** reduced_resolution_map = reduce_resolution(reshaped_map, real_resolution, desired_resolution, height, width);
    plot_org_map(reduced_resolution_map, reduced_height, reduced_width, origin[0], origin[1]);

    CoveragePath my_coverage_path(reduced_resolution_map, reduced_height, reduced_width);
    my_coverage_path.createWeights(origin[0], origin[1]);
    my_coverage_path.createCoveragePath();


    grid * currentGrid = my_coverage_path.coveragePath.getHead();
    MoveBaseClient ac("move_base", true);
    ac.waitForServer(ros::Duration(5.0));
    while (currentGrid != nullptr && ros::ok()) {
        // sending goal actions to move_base
        
        move_base_msgs::MoveBaseGoal goal;
        goal.target_pose.header.frame_id = "map";
        goal.target_pose.header.stamp = ros::Time::now();
        goal.target_pose.pose = calculatePose(currentGrid->x, currentGrid->y, origin[0], origin[1], desired_resolution);

        std::cout << "Heading towards grid [" << currentGrid->x << "][" << currentGrid->y << "]: ";
        std::cout << "(" << goal.target_pose.pose.position.x << "," << goal.target_pose.pose.position.y << ")" << std::endl;
        // plot_org_map(reduced_resolution_map, reduced_height, reduced_width, currentGrid->x, currentGrid->y);

        ac.sendGoal(goal);
        

        ros::Rate rate(5);
        while (ros::ok()) {
            if (calculateDistance(robot_pose.position.x, robot_pose.position.y, goal.target_pose.pose.position.x, goal.target_pose.pose.position.y) < mu) {
                std::cout << "Goal Reached" << std::endl;
                break;
            }
            ros::spinOnce();
            rate.sleep();
        }
        currentGrid = currentGrid->next;
    }


    // deleting the dynamically allocated arrays
    for (int i = 0; i < height; i++) {
        delete[] reshaped_map[i];
    }
    delete[] reshaped_map;

    return 0;
}

// callback for /map topic
void map_callback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    occupancy_grid_map.data = msg->data;
    occupancy_grid_map.header = msg->header;
    occupancy_grid_map.info = msg->info;
    updated = true;
}

// callback for /odom topic
void odom_callback(const nav_msgs::Odometry::ConstPtr& msg)
{
    robot_pose= msg->pose.pose;
}

geometry_msgs::Pose calculatePose(int x, int y, int x_offset, int y_offset, float resolution) {
    geometry_msgs::Pose pose;
    pose.position.y = float ((x - x_offset)) * resolution + 0.2;
    pose.position.x = float ((y - y_offset)) * resolution + 0.2;
    float headingAngle = atan2(pose.position.y - robot_pose.position.y, pose.position.x - robot_pose.position.x);
    tf::Quaternion headingAngle_quat = tf::createQuaternionFromYaw(headingAngle);
    pose.orientation.x = headingAngle_quat.x();
    pose.orientation.y = headingAngle_quat.y();
    pose.orientation.z = headingAngle_quat.z();
    pose.orientation.w = headingAngle_quat.w();

    return pose;
}



