#pragma once
#include <algorithm>
#include <boost/archive/text_iarchive.hpp>
#include <boost/serialization/vector.hpp>
#include <fstream>
#include <iostream>
#include <iterator>
#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/cloud_viewer.h>
#include <vector>
#include "omp.h"

/**
* Convert libfreenect data to Point Cloud Library format pcd
*/
template <typename PointT> class Freenect2Pcl {
public:
/**
* Constructor of Freenect2Pcl object
* @param[in] depthfile name of the depth file
* @param[in] rgbfile name of the rgbfile
*/
    Freenect2Pcl(std::string depthfile, std::string rgbfile)
        : _depthfile(depthfile), _rgbfile(rgbfile) {
        {
            std::ifstream ifs(_rgbfile);
            boost::archive::text_iarchive ia(ifs);
            ia &rgb;
        }
        {
            std::ifstream ifs(_depthfile);
            boost::archive::text_iarchive ia(ifs);
            ia &_depth;
        }
    }
/**
* Default Destructor
*/

    ~Freenect2Pcl() {}

/**
* Template for point cloud conversion. Use this function to return a
* pcd formatted point cloud.
* @param[in] distance maximum distance to get from depth data
* @param[in] colored boolean value to apply rgb data or not
*/
    typename pcl::PointCloud<PointT>::Ptr getPointCloud(int distance,
                                                        bool colored) {
        std::cerr << "Reading Point Cloud Data... ";

        int depth_width = 640;
        int depth_height = 480;

        // create the empty Pointcloud
        boost::shared_ptr<pcl::PointCloud<PointT>> cloud(
                    new pcl::PointCloud<PointT>);

        // allow infinite values for points coordinates
        cloud->is_dense = false;

        // set camera parameters for kinect
        double focal_x_depth = 585.187492217609;  // 5.9421434211923247e+02;
        double focal_y_depth = 585.308616340665;  // 5.9104053696870778e+02;
        double center_x_depth = 322.714077555293; // 3.3930780975300314e+02;
        double center_y_depth = 248.626108676666; // 2.4273913761751615e+02;

        float bad_point = std::numeric_limits<float>::quiet_NaN();
#pragma omp parallel for
        for (unsigned int y = 0; y < depth_height; ++y)
            for (unsigned int x = 0; x < depth_width; ++x) {
                PointT ptout;
                uint16_t dz = _depth[y * depth_width + x];
                if (abs(dz) < distance) {
                    Eigen::Vector3d ptd((x - center_x_depth) * dz / focal_x_depth,
                                        (y - center_y_depth) * dz / focal_y_depth, dz);
                    // assign output xyz

                    ptout.x = ptd.x() * 0.001f;
                    ptout.y = ptd.y() * 0.001f;
                    ptout.z = ptd.z() * 0.001f;

                    if (colored) {
                        uint8_t r = rgb[(y * depth_width + x) * 3];
                        uint8_t g = rgb[(y * depth_width + x) * 3 + 1];
                        uint8_t b = rgb[(y * depth_width + x) * 3 + 2];

                        ptout.rgba = pcl::PointXYZRGB(r, g, b).rgba; // assign color

                    } else
                        ptout.rgba = pcl::PointXYZRGB(0, 0, 0).rgba;
#pragma omp critical
                    cloud->points.push_back(ptout); // assigns point to cloud
                }
            }
        cloud->height = 480;
        cloud->width = 640;

        std::cerr << "DONE!" << std::endl;
        return (cloud);
    }

private:
    std::string _depthfile;
    std::string _rgbfile;
    std::vector<uint16_t> _depth;
    std::vector<uint8_t> rgb;
};
