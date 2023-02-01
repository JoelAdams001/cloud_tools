#define BOOST_BIND_NO_PLACEHOLDERS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include "rclcpp/rclcpp.hpp"
#include <sensor_msgs/msg/point_cloud2.hpp>

using std::placeholders::_1;
class Voxelizer : public rclcpp::Node
{
  public:
    Voxelizer()
    : Node("Voxelizer")
    {
      this->declare_parameter<std::string>("cloud_topic", "camera/depth/color/points");
      this->get_parameter("cloud_topic", cloud_topic_);
      
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, 10, std::bind(&Voxelizer::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("voxelized_cloud", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2& cloud_in) const
        {
            pcl::PCLPointCloud2::Ptr cloud_raw (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr cloud_filtered (new pcl::PCLPointCloud2 ());
            sensor_msgs::msg::PointCloud2 cloud_ros;

            // Convert cloud from ROS msg to PCL msg
            pcl_conversions::toPCL(cloud_in, *cloud_raw);

            // Create the filtering object
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (cloud_raw);
            sor.setLeafSize (0.01f, 0.01f, 0.01f);
            sor.filter (*cloud_filtered);

            // Convert back to ROS msg
            pcl_conversions::fromPCL(*cloud_filtered, cloud_ros);
            
            // Publish cloud
            publisher_->publish(cloud_ros);
        }
    std::string cloud_topic_;
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Voxelizer>());
  rclcpp::shutdown();
  return 0;
}
