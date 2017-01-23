#include "feature.h"
#include "filter.h"
#include "raspiconnect.h"
#include "registration.h"
#include "visualization.h"
#include <pcl/point_types.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp.h>

const int distance = 7000;
const uint16_t point_count = 3;

int main() {
/*
    RaspberryCon raspi1; // BACK *
    RaspberryCon raspi2; // FRONT *
    RaspberryCon raspi3; // RIGHT *
    RaspberryCon raspi4; // LEFT *

    std::cerr << "Getting data from Kinect 1...";
    raspi1.init("pi", "192.168.12.237", "raspberry");
    raspi1.connect();
    raspi1.runKinect("depth_back", "rgb_back");
    raspi1.getData();

    raspi1.disconnect();
    std::cerr << " DONE!" << std::endl;
    std::cerr << "Getting data from Kinect 2...";
    raspi2.init("pi", "192.168.12.12", "raspberry");
    raspi2.connect();
    raspi2.runKinect("depth_front", "rgb_front");
    raspi2.getData();

    raspi2.disconnect();
    std::cerr << " DONE!" << std::endl;
    std::cerr << "Getting data from Kinect 3...";
    raspi3.init("pi", "192.168.12.206", "raspberry");
    raspi3.connect();
    raspi3.runKinect("depth_right", "rgb_right");
    raspi3.getData();

    raspi3.disconnect();
    std::cerr << " DONE!" << std::endl;

    std::cerr << "Getting data from Kinect 4...";
    raspi4.init("pi", "192.168.12.246", "raspberry");
    raspi4.connect();
    raspi4.runKinect("depth_left", "rgb_left");
    raspi4.getData();
    raspi4.disconnect();
    std::cerr << " DONE!" << std::endl;
*/

    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_back;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_left;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_right;
    boost::shared_ptr<pcl::PointCloud<pcl::PointXYZRGB>> cloud_front;


    boost::shared_ptr<Filter> filter = boost::make_shared<Filter>();
    boost::shared_ptr<Feature> feature = boost::make_shared<Feature>();
    boost::shared_ptr<Registration> registration =
            boost::make_shared<Registration>();

    // Initialize Freenect2Pcl Objects
    Freenect2Pcl<pcl::PointXYZRGB> c_back("depth_back", "rgb_back");
    Freenect2Pcl<pcl::PointXYZRGB> c_left("depth_left", "rgb_left");
    Freenect2Pcl<pcl::PointXYZRGB> c_right("depth_right", "rgb_right");
    Freenect2Pcl<pcl::PointXYZRGB> c_front("depth_front", "rgb_front");

    cloud_back = c_back.getPointCloud(distance, true);
    cloud_left = c_left.getPointCloud(distance, true);
    cloud_right = c_right.getPointCloud(distance, true);
    cloud_front = c_front.getPointCloud(distance, true);

    boost::shared_ptr<Feature::myCloud> Cloud1 =
            boost::make_shared<Feature::myCloud>();

    pcl::io::savePCDFileASCII("serhat_back.pcd", *cloud_back);
    pcl::io::savePCDFileASCII("serhat_left.pcd", *cloud_left);
    pcl::io::savePCDFileASCII("serhat_right.pcd", *cloud_right);
    pcl::io::savePCDFileASCII("serhat_front.pcd", *cloud_front);


    cloud_back = filter->applyFilters(cloud_back, 0.8, 1.5, 0.001f, 0.02, 20);
    cloud_left = filter->applyFilters(cloud_left, 0.8, 1.3, 0.001f, 0.02, 20);
    cloud_right = filter->applyFilters(cloud_right, 0.8, 1.3, 0.001f, 0.02, 20);
    cloud_front = filter->applyFilters(cloud_front, 0.8, 1.5, 0.001f, 0.02, 20);

    pcl::io::savePCDFileASCII("fserhat_back.pcd", *cloud_back);
    pcl::io::savePCDFileASCII("fserhat_left.pcd", *cloud_left);
    pcl::io::savePCDFileASCII("fserhat_right.pcd", *cloud_right);
    pcl::io::savePCDFileASCII("fserhat_front.pcd", *cloud_front);


    Eigen::Affine3f transform_1 = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_2 = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_3 = Eigen::Affine3f::Identity();
    Eigen::Affine3f transform_4 = Eigen::Affine3f::Identity();

    transform_1.translation() << -0.10, 0.03, 2.4-0.070; //done
    transform_2.translation() << -0.16623, -0.00, 2.4; //done
    transform_3.translation() << 0.1, 0.002, 2.95; //done
    transform_4.translation() << 0.060, -0.045, 0.68; //done

    for (size_t i = 0; i < cloud_front->points.size (); ++i)
    {
        cloud_front->points[i].x = -cloud_front->points[i].x;
        cloud_front->points[i].z = -cloud_front->points[i].z;
    }
    for (size_t i = 0; i < cloud_left->points.size (); ++i)
    {
        cloud_left->points[i].x = -cloud_left->points[i].x;
        cloud_left->points[i].z = -cloud_left->points[i].z;
    }

    pcl::transformPointCloud (*cloud_front, *cloud_front, transform_1);
    pcl::transformPointCloud (*cloud_left, *cloud_left, transform_2);

    *cloud_left += *cloud_right;
    cloud_left = filter->matrixTransform(cloud_left,'Y',M_PI/2,0,true);

    for (size_t i = 0; i < cloud_left->points.size (); ++i)
    {
        cloud_left->points[i].z = -cloud_left->points[i].z;
    }

    pcl::transformPointCloud (*cloud_left, *cloud_left, transform_3);
    *cloud_front += *cloud_back;
    pcl::transformPointCloud (*cloud_front, *cloud_front, transform_4);

    pcl::io::savePCDFileASCII("serhat_depthrgb_back.pcd", *cloud_back);
    pcl::io::savePCDFileASCII("serhat_depthrgb_left.pcd", *cloud_left);
    pcl::io::savePCDFileASCII("serhat_depthrgb_right.pcd", *cloud_right);
    pcl::io::savePCDFileASCII("serhat_depthrgb_front.pcd", *cloud_front);

    *cloud_front += *cloud_left;


    pcl::io::savePCDFileASCII("model.pcd", *cloud_front);
    auto model_normals= feature->estimateSurfaceNormalsOMP(cloud_front,0.01f);
//    pcl::PointCloud<pcl::PointXYZ>::Ptr mesh_cloud (new pcl::PointCloud<pcl::PointXYZ>);
 //   pcl::io::loadPCDFile("mesh.pcd",*mesh_cloud);
 //   auto cloud_normals = feature->estimateSurfaceNormalsOMP(mesh_cloud,0.01f);
    pcl::PolygonMesh mesh =
            registration->greedyTriangulation(cloud_front,model_normals);
    pcl::io::saveVTKFile("mesh.vtk", mesh);

    return 0;
}
