//
// Created by redwan on 12/2/20.
//

#ifndef AREA_COVERAGE_RECEEDING_HORIZON_H
#define AREA_COVERAGE_RECEEDING_HORIZON_H
#include <memory>
#include <vector>
#include <ros/ros.h>
#include <trajectory_msgs/JointTrajectory.h>
#include <geometry_msgs/PoseArray.h>
#include <visualization_msgs/Marker.h>


#define MAX_NUM_ROBOTS 10

using namespace std;
class ReceedingHorizon: public enable_shared_from_this<ReceedingHorizon>
{
    using rhPtr = shared_ptr<ReceedingHorizon>;
    using vecPoint = vector<geometry_msgs::Point>;
public:
    ReceedingHorizon()
    {
        HorizonView = nh.subscribe("/multiquad/trajs", 1000, &ReceedingHorizon::mulitquadCallback, this);
        PathView = nh.subscribe("/multiquad/paths", 1000, &ReceedingHorizon::pathCallback, this);

        marker_pub = nh.advertise<visualization_msgs::Marker>("receding_horizon", 10);
        points.action = line_strip.action = visualization_msgs::Marker::ADD;
        points.header.frame_id = line_strip.header.frame_id = "world";
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;
        points.type = visualization_msgs::Marker::POINTS;
        points.scale.x = 0.03;
        points.scale.y = 0.03;
        points.color.a = 1.0;
        line_strip.scale.x = 0.02;
        line_strip.scale.y = 0.02;
        line_strip.scale.z = 0.02;
        line_strip.color.g = 1.0;
        line_strip.color.a = 0.50;
        line_strip.ns = "horizon";
        points.ns = "waypoints";
        robots.resize(MAX_NUM_ROBOTS);
        enabled = false;

        if (! nh.getParam("altitude", altitude))
            altitude = 1.0;
    }
    rhPtr getPtr()
    {
        return shared_from_this();
    }

    void pathCallback(const geometry_msgs::PoseArray::ConstPtr& msg)
    {
        points.id  = atoi(msg->header.frame_id.c_str());
        points.header.stamp = ros::Time::now();
        points.color.r = 0.0f;
        points.color.g = 0.0f;
        points.color.b = 0.0f;
        switch (points.id) {
            case 0: points.color.r = 1.0f; break;
            case 1: points.color.g = 1.0f; break;
            case 2: points.color.b = 1.0f; break;
            default:points.color.r = 1.0f;
                break;
        }
        points.points.clear();
        for (auto &pose: msg->poses)
        {
            points.points.push_back(pose.position);
        }

        marker_pub.publish(points);
    }

    void mulitquadCallback(const trajectory_msgs::JointTrajectory::ConstPtr& msg)
    {

        auto frame_id = msg->header.frame_id;
        line_strip.header.stamp = ros::Time::now();
        line_strip.id = atoi(frame_id.c_str());
        line_strip.points.clear();


        int count = 0;
        for(auto &point: msg->points)
        {
            geometry_msgs::Point p;
            p.x = point.positions[0];
            p.y = point.positions[1];
            p.z = altitude;

            yaw[line_strip.id ] = point.effort[0];
            line_strip.points.push_back(p);
            if(count++ == 0)
            {
                robots[line_strip.id] = p;
            }
        }

        if(line_strip.points.size() %2 != 0)
        {
            auto N = line_strip.points.size() - 1;
            auto last_point = line_strip.points[N];
            line_strip.points.push_back(last_point);
        }

        marker_pub.publish(line_strip);
        ROS_INFO_STREAM(*msg);
        enabled = true;
    }

    geometry_msgs::Point getCoord(int robot_id)
    {
        return robots[robot_id];
    }
    bool enabled;
    double yaw[MAX_NUM_ROBOTS];

private:
    visualization_msgs::Marker points, line_strip;
    ros::NodeHandle nh;
    ros::Publisher marker_pub;
    ros::Subscriber HorizonView, PathView;
    vecPoint robots;
    double altitude;

};
#endif //AREA_COVERAGE_RECEEDING_HORIZON_H
