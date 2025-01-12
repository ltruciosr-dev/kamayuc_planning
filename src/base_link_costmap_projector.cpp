//https://github.com/osrf/subt_hello_world/blob/master/subt_solution_launch/src/base_link_costmap_projector.cpp
#include <ros/ros.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

int main(int argc, char *argv[])
{
    ros::init(argc, argv, "flattening_node");
    ros::NodeHandle nh;

    // Get the robot's name
    ros::NodeHandle private_nh("~");
    std::string robot_frame;
    private_nh.param<std::string>("frame", robot_frame, "base_link");

    // Define the parent and child frames of the new transform.
    // In this case, we are creating a new transform between the
    // map frame and "fake" frame. The "fake" frame is the robot's
    // base link frame in move_base's costmap plane
    const std::string parent = "map";
    const std::string child = "fake";
    const std::string sensor = "zed2_left_camera_optical_frame";
    // Create objects for receiving and publishing transforms
    tf2_ros::Buffer tfBuffer;
    tf2_ros::TransformListener tfListener(tfBuffer);
    tf2_ros::TransformBroadcaster br;

    // Publish [fake -> sensor_fake] static frame (as same as [base_link -> zed2_left_camera_optical_frame])
    static tf2_ros::StaticTransformBroadcaster static_broadcaster;
    geometry_msgs::TransformStamped realSensorTf, fakeSensorTf;

    realSensorTf = tfBuffer.lookupTransform(robot_frame, sensor, ros::Time(0), ros::Duration(3.0));

    fakeSensorTf = realSensorTf;
    fakeSensorTf.header.stamp = ros::Time::now();
    fakeSensorTf.header.frame_id = "fake";
    fakeSensorTf.child_frame_id = "sensor_fake";
    static_broadcaster.sendTransform(fakeSensorTf);

    ros::Rate rate(10.0);
    while (nh.ok())
    {
        try
        {
            geometry_msgs::TransformStamped outTf, inTf;
            // Get the original map -> base_link transform
            inTf = tfBuffer.lookupTransform(parent, robot_frame, ros::Time(0), ros::Duration(0.5));

            // Define the new transform
            outTf.header.stamp = ros::Time::now();
            outTf.header.frame_id = parent;
            outTf.child_frame_id = child;

            // Remove the roll and pitch from the original map -> base_link transform.
            // We only want to keep the yaw since this is in the xy plane, which is the
            // plane of the costmap for move_base
            tf2::Quaternion q, q0;
            tf2::fromMsg(inTf.transform.rotation, q);
            tf2::Matrix3x3 m(q);
            double roll, pitch, yaw;
            m.getRPY(roll, pitch, yaw);
            q0.setRPY(0, 0, yaw);

            // Apply the quaternion that only contains the yaw to the new transform
            outTf.transform.rotation = tf2::toMsg(q0);

            // Copy the position of base_link over to the new transform, removing
            // the height of the base link since move_base's costmap has a height of 0
            outTf.transform.translation.x = inTf.transform.translation.x;
            outTf.transform.translation.y = inTf.transform.translation.y;
            outTf.transform.translation.z = 0.0;

            // Publish the transform
            br.sendTransform(outTf);

            rate.sleep();
        }
        catch (tf2::TransformException &ex)
        {
            ROS_WARN("%s", ex.what());
            ros::Duration(0.1).sleep();
        }
    }
}