#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_cloud.h>
#include <tf2/exceptions.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <cmath>
#include <string>
float flat_height;
float slope_threshold;
int scale_cloud;

std::string rover_frame;
std::string flat_frame;
std::string new_frame;
std::string points;

ros::Publisher filtered_pc_pub;
tf2_ros::Buffer tf_buffer;

// The callback method for the subscriber.
// Receives the original point cloud data, filters it, and then
// publishes this filtered point cloud to a new topic
void filter_pc(const sensor_msgs::PointCloud2::ConstPtr &pc);

int main(int argc, char *argv[])
{
    // Start node
    ros::init(argc, argv, "pc_filter");
    ros::NodeHandle nh;

    // Set up parameters.
    // The slope parameter is the upper bound of a height to distance ratio that determines
    // whether a point in a point cloud at a certain location is an obstacle or not
    ros::NodeHandle private_nh("~");
    private_nh.param<float>("flat_height", flat_height, 0.05);
    private_nh.param<float>("slope", slope_threshold, 1.0);
    private_nh.param<int>("scale", scale_cloud, 1);
    private_nh.param<std::string>("frame", rover_frame, "base_link");
    private_nh.param<std::string>("points", points, "zed2/point_cloud/cloud_registered");

    // Define transform frames
    flat_frame = "map";
    rover_frame = "base_link";
    new_frame = "fake";

    // Set up a publisher for the filtered point cloud
    // and initialize the buffer so we can lookup transforms
    filtered_pc_pub = nh.advertise<sensor_msgs::PointCloud2>("points_filtered", 1);
    tf2_ros::TransformListener tf_listener(tf_buffer);

    // Subscribe to the original point cloud topic
    ros::Subscriber original_pc_sub = nh.subscribe(points, 1, &filter_pc);

    // Start listening for data and trigger the callback when possible
    ros::spin();
}

static void subsample_data(const sensor_msgs::PointCloud2::ConstPtr &pc,
                           sensor_msgs::PointCloud2::Ptr &pc_filtered)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromROSMsg(*pc, *cloud);

    cloud_filtered->width = cloud->width / scale_cloud;
    cloud_filtered->height = cloud->height / scale_cloud;
    cloud_filtered->points.resize(cloud_filtered->width * cloud_filtered->height);
    size_t total = 0;
    try
    {
        for (int x = 0; x < cloud_filtered->width; ++x)
        {
            for (int y = 0; y < cloud_filtered->height; ++y)
            {
                total++;
                cloud_filtered->at(x, y) = cloud->at(x * scale_cloud, y * scale_cloud);
            }
        }
        pcl::toROSMsg(*cloud_filtered, *pc_filtered);
        pc_filtered->header = pc->header;
    }
    catch (const std::exception &e)
    {
        std::cout << e.what() << std::endl;
        std::cout << "uncomplete" << total << std::endl;
    }
}

void filter_pc(const sensor_msgs::PointCloud2::ConstPtr &pc)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr body(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PointCloud<pcl::PointXYZ>::Ptr publishBody(new pcl::PointCloud<pcl::PointXYZ>);
    sensor_msgs::PointCloud2 result;
    sensor_msgs::PointCloud2::Ptr pc_filtered(new sensor_msgs::PointCloud2);
    subsample_data(pc, pc_filtered);

    geometry_msgs::PointStamped origin;
    geometry_msgs::TransformStamped transformZedToMap, transformRoverToMap;

    try
    {
        // Find the transform between the real and fake sensor frame,
        // and then apply this transformation to the original point cloud
        // so that we have point cloud data in the fake sensor frame
        // header_parent_link
        std::string pc_frame = pc_filtered->header.frame_id;
        if (pc_frame.substr(0, 1) == "/")
            pc_frame = pc_frame.substr(1);

        // Transforms: map -> zed ; map -> rover
        transformZedToMap = tf_buffer.lookupTransform(pc_frame, flat_frame,
                                                      ros::Time(0), ros::Duration(0.5));
        transformRoverToMap = tf_buffer.lookupTransform(rover_frame, flat_frame,
                                                        ros::Time(0), ros::Duration(0.5));

        geometry_msgs::PointStamped tempOrigin;
        tempOrigin.point.x = 0;
        tempOrigin.point.y = 0;
        tempOrigin.point.z = flat_height;
        tempOrigin.header.frame_id = rover_frame;
        tf2::doTransform(tempOrigin, origin, transformRoverToMap);
        tf2::doTransform(*pc_filtered, result, transformZedToMap);
    }
    catch (tf2::TransformException &ex)
    {
        ROS_WARN("%s", ex.what());
        return;
    }

    // Filter the point cloud data.
    // If a point's slope in the point cloud is less than the slope_threshold
    // (the slope is calculated relative to a point's neighbor in the point cloud),
    // we "delete" this point so that move_base doesn't interpret it as an obstacle.
    // We need to do this since move_base is 2D and our world is 3D
    // with varying terrain height
    pcl::fromROSMsg(result, *body);
    pcl::fromROSMsg(*pc_filtered, *publishBody);

    for (size_t x = 0; x < body->width; ++x)
    {
        geometry_msgs::PointStamped last_point;
        last_point.point.x = 0;
        last_point.point.y = 0;
        last_point.point.z = 0;
        for (size_t y = 0; y < body->height; ++y)
        {
            geometry_msgs::PointStamped transformed;
            pcl::PointXYZ actual_point = body->at(x, y);
            float diff_z = actual_point.z - last_point.point.z;
            float diff_position = sqrt(pow(actual_point.x - last_point.point.x, 2) +
                                       pow(actual_point.y - last_point.point.y, 2));
            if (abs(diff_position) < .0001)
            {
                continue;
            }
            float slope = diff_z / diff_position;

            last_point.point.x = actual_point.x;
            last_point.point.y = actual_point.y;
            last_point.point.z = actual_point.z;
            if (slope == INFINITY || slope == NAN)
            {
                continue;
            }
            else if (slope < slope_threshold)
            {
                actual_point.x = INFINITY;
                actual_point.y = INFINITY;
                actual_point.z = origin.point.z;
            }
            publishBody->at(x, y) = actual_point;
        }
    }

    // Publish the filtered point cloud so move_base can use it
    pcl::toROSMsg(*publishBody, result);
    result.header.frame_id = "sensor_fake";
    filtered_pc_pub.publish(result);
};