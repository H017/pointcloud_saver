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
#include <pcl/io/openni_grabber.h>

typedef pcl::PointCloud<pcl::PointXYZRGBA> PointCloud;

ros::NodeHandle* nh;
std::string savePath, targetFrame;
tf::TransformListener* tfListener;
pcl::Grabber* interface;
PointCloud::Ptr currentCloud;
ros::Time currentPointCloudTime;

bool save_pc_callback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
    tfListener->waitForTransform(targetFrame, (*currentCloud).header.frame_id, ros::Time(0), ros::Duration(5.0));
    PointCloud pc;
    pcl_ros::transformPointCloud(targetFrame, *currentCloud, pc, *tfListener); 

    std::stringstream ss;
    ss << savePath << "/pc_" << currentPointCloudTime << ".pcd";      
    pcl::io::savePCDFile (ss.str (), pc, true); 

    return true;
}

void cloudCallback (const PointCloud::ConstPtr &cloud)
{
    currentPointCloudTime = ros::Time::now();
    pcl::copyPointCloud(*cloud, *currentCloud);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "pointcloud_saver");
    nh = new ros::NodeHandle;
    ros::NodeHandle n("~");

    // Params
    nh->param<std::string>("save_path", savePath, "pointclouds");
    nh->param<std::string>("target_frame", targetFrame, "/map");
        
    // Openni
    interface = new pcl::OpenNIGrabber();
    boost::function<void (const PointCloud::ConstPtr&)> f = boost::bind (cloudCallback, _1);
    interface->registerCallback (f);
    currentCloud = PointCloud::Ptr(new PointCloud);
    interface->start ();

    // TF + Service
    tfListener = new tf::TransformListener;
    ros::ServiceServer service = n.advertiseService("save_pointcloud", save_pc_callback);
    ros::spin();
    
    interface->stop();
}