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

class Feature {
public:
    struct myCloud {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr points;
        pcl::PointCloud<pcl::Normal>::Ptr normals;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr keypoints;
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr local_descriptor;
        pcl::PointCloud<pcl::VFHSignature308>::Ptr global_descriptor;
    };

    pcl::PointCloud<pcl::Normal>::Ptr
    estimateSurfaceNormals(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                           float radius);


    pcl::PointCloud<pcl::Normal>::Ptr
    estimateSurfaceNormalsOMP(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                              float radius);


    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    getSiftKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                     const float min_scale, const int n_octaves,
                     const int n_scales_per_octave, const float min_contrast);

    pcl::PointCloud<pcl::FPFHSignature33>::Ptr
    getLocalDescriptor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                       const pcl::PointCloud<pcl::Normal>::Ptr &normals,
                       const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints,
                       float radius);

    pcl::PointCloud<pcl::VFHSignature308>::Ptr
    getGlobalDescriptor(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &points,
                        const pcl::PointCloud<pcl::Normal>::Ptr &normals);

    boost::shared_ptr<Feature::myCloud>
    applyFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

private:
};
