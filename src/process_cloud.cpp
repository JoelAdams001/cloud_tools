#define BOOST_BIND_NO_PLACEHOLDERS
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/voxel_grid.h>
#include "pcl_conversions/pcl_conversions.h"
#include <pcl/kdtree/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
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
      
      this->declare_parameter<std::string>("frame_id", "camera_depth_optical_frame");
      this->get_parameter("frame_id", frame_id_);
      
      this->declare_parameter<double>("cluster_tolerance", 0.2);
      this->get_parameter("cluster_tolerance", cluster_tolerance_);
      
      this->declare_parameter<double>("max_cluster_size", 20000);
      this->get_parameter("max_cluster_size", max_cluster_size_);
      
      subscription_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      cloud_topic_, 10, std::bind(&Voxelizer::topic_callback, this, _1));
      
      publisher_ = this->create_publisher<sensor_msgs::msg::PointCloud2>("clustered_cloud", 10);
    }

  private:
    void topic_callback(const sensor_msgs::msg::PointCloud2& cloud_in) const
        {
            pcl::PCLPointCloud2::Ptr cloud_raw (new pcl::PCLPointCloud2 ());
            pcl::PCLPointCloud2::Ptr cloud_filtered2 (new pcl::PCLPointCloud2 ());
            sensor_msgs::msg::PointCloud2 cloud_ros;

            // Convert cloud from ROS msg to PCL msg
            pcl_conversions::toPCL(cloud_in, *cloud_raw);

            // Create the filtering object
            pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
            sor.setInputCloud (cloud_raw);
            sor.setLeafSize (0.05f, 0.05f, 0.05f);
            sor.filter (*cloud_filtered2);

            // Segmentation
            // Convert from pointcloud2 to pointcloud in order to input to KdTree
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);
            pcl::fromPCLPointCloud2 (*cloud_filtered2, *cloud_filtered);
            
            pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
            tree->setInputCloud (cloud_filtered);

            std::vector<pcl::PointIndices> cluster_indices;
            pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
            ec.setClusterTolerance (0.1); // 2cm
            ec.setMinClusterSize (10);
            ec.setMaxClusterSize (20000);
            ec.setSearchMethod (tree);
            ec.setInputCloud (cloud_filtered);
            ec.extract (cluster_indices);
            
            //Print out info on extract_clusters
            int j = 0;
            
            //pcl::Pointcloud<pcl::PointXYZ>::Ptr all_clusters (new pcl::Pointcloud<pcl::PointXYZ>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered_color (new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl::copyPointCloud(*cloud_filtered, *cloud_filtered_color);
            for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
            {
                for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit){
                cloud_filtered_color->points[*pit].r = 100-j*10;
                cloud_filtered_color->points[*pit].b = 100+j*50;
                cloud_filtered_color->points[*pit].g = 0+j*5;
                cloud_cluster->push_back ((*cloud_filtered_color)[*pit]); //*
                cloud_cluster->width = cloud_cluster->size ();
                cloud_cluster->height = 1;
                cloud_cluster->is_dense = true;
                }
                std::cout << "PointCloud representing the Cluster: " << cloud_cluster->size () << " data points.  cluster count: " << j << std::endl;
                j++;
            }
            
            // Convert back to pcl::Pointcloud2 and then to ROS msg
            pcl::toPCLPointCloud2 (*cloud_cluster, *cloud_filtered2);
            pcl_conversions::fromPCL(*cloud_filtered2, cloud_ros);
            cloud_ros.header.frame_id = frame_id_;
            
            // Publish cloud
            publisher_->publish(cloud_ros);
        }
    std::string cloud_topic_, frame_id_;
    double cluster_tolerance_, max_cluster_size_;
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
