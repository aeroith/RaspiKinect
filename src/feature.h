#pragma once
#include <boost/shared_ptr.hpp>
#include <iostream>
#include <omp.h>
#include <pcl/features/fpfh_omp.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/vfh.h>
#include <pcl/features/vfh.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>

/**
* A Feature class that holds many useful functions
*/
class Feature {
public:

/**
* A point cloud structure to hold points, normals, keypoints, local and global descriptors
*/
    struct myCloud {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr local_descriptor;
        pcl::PointCloud<pcl::VFHSignature308>::Ptr global_descriptor;
    };
/**
* Estimate Surface Normals using kdtree
* @param[in] cloud input point cloud
* @param[in] radius radius in meters
* \deprecated Use estimateSurfaceNormalsOMP instead for better performance
*/
    pcl::PointCloud<pcl::Normal>::Ptr
    estimateSurfaceNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                           float radius);

/**
* Estimate Surface Normals using kdtree.
* @param[in] cloud input point cloud
* @param[in] radius radius in meters
*/
    pcl::PointCloud<pcl::Normal>::Ptr
    estimateSurfaceNormalsOMP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                              float radius);

/**
* Compute keypoints using a 3D adaptation of David Lowe’s SIFT keypoint detector.
* Returns a pointer to XYZRGB Point Cloud.
* @param[in] cloud input point cloud
* @param[in] min_scale the standard deviation of the smallest scale in the scale space
* @param[in] n_octaves the number of octaves (i.e. doublings of scale) to compute
* @param[in] n_scales_per_octave the number of scales to compute within each octave
* @param[in] min_contrast the minimum contrast required for detection
*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    getSiftKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                     const float min_scale, const int n_octaves,
                     const int n_scales_per_octave, const float min_contrast);
/**
* Compute Fast Point Feature Histograms. The default FPFH implementation uses 11 binning subdivisions 
* (e.g., each of the four feature values will use this many bins from its value interval), 
* and a decorrelated scheme which results in a 33-byte array of float values. 
* These are stored in a pcl::FPFHSignature33 point type.
* @param[in] points input point cloud
* @param[in] normals point cloud normals
* @param[in] keypoints computed keypoints
* @param[in] radius radius in meters
*/
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
    getLocalDescriptor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                       const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
                       float radius);
/**
* Viewpoint Feature Histogram ([VFH]) descriptor, a novel representation for point clusters 
* for the problem of Cluster (e.g., Object) Recognition and 6DOF Pose Estimation. 
* Uses a similar approach like Fast Point Feature Histogram but much faster.
* @param[in] points input point cloud
* @param[in] normals point cloud normals
*/
    pcl::PointCloud<pcl::VFHSignature308>::Ptr
    getGlobalDescriptor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                        const pcl::PointCloud<pcl::Normal>::Ptr &normals);
/**
* Apply the feature extraction to the given myCloud struct. 
* Returns a pointer to the myCloud struct.
* @param[in] cloud input point cloud
*/
    boost::shared_ptr<Feature::myCloud>
    applyFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

private:
};
