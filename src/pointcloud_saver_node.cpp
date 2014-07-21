#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <std_msgs/Header.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <tf/transform_listener.h>

typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

ros::NodeHandle* nh;
std::string savePath, targetFrame;
tf::TransformListener* tfListener;

bool save_pc_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    PointCloud::ConstPtr pc = ros::topic::waitForMessage<PointCloud >("/camera/depth_registered/points", *nh, ros::Duration(3.0));
    std_msgs::Header header = pcl_conversions::fromPCL(pc->header);

    if (!pc)
    {
        ROS_ERROR("Could not retrieve point cloud");
        return false;
    }

    PointCloud cloud;
    try
    {
        tfListener->waitForTransform(targetFrame, header.frame_id, header.stamp, ros::Duration(5.0));
        pcl_ros::transformPointCloud(targetFrame, *pc, cloud, *tfListener);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s",ex.what());
        return false;
    }

    std::stringstream ss;
    ss << savePath << "/pc_" << pc->header.stamp << ".pcd";      
    pcl::io::savePCDFile (ss.str (), cloud, true); 
  
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_saver");
    nh = new ros::NodeHandle;
    ros::NodeHandle n("~");
    tfListener = new tf::TransformListener;

    nh->param<std::string>("save_path", savePath, "pointclouds");
    nh->param<std::string>("target_frame", targetFrame, "/map");

    ros::ServiceServer service = n.advertiseService("save_pointcloud", save_pc_callback);

    ros::spin();
}