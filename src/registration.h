#pragma once
#include <Eigen/Geometry>
#include <boost/make_shared.hpp>
#include <boost/shared_ptr.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/common/common.h>
#include <pcl/common/io.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/pfh.h>
#include <pcl/features/vfh.h>
#include <pcl/filters/approximate_voxel_grid.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/io/vtk_io.h>
#include <pcl/keypoints/harris_3d.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/keypoints/sift_keypoint.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/registration/correspondence_rejection_features.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <pcl/registration/ndt.h>
#include <pcl/registration/sample_consensus_prerejective.h>
#include <pcl/search/impl/search.hpp>
#include <pcl/search/kdtree.h>
#include <pcl/surface/poisson.h>
#include <pcl/surface/reconstruction.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <string.h>
#include <string>
#include "feature.h"
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/surface/gp3.h>
class Registration : public Feature {
public:
    Eigen::Matrix4f computeInitialAlignment(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
            const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &source_descriptors,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
            const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &target_descriptors,
            float min_sample_distance, float max_correspondence_distance,
            int nr_iterations);
    Eigen::Matrix4f
    refineAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                    const Eigen::Matrix4f &initial_alignment,
                    float max_correspondence_distance,
                    float outlier_rejection_threshold,
                    float transformation_epsilon, int max_iterations);


    pcl::PolygonMesh
    poissonReconstruct(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                       const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    applyICP(const boost::shared_ptr<Feature::myCloud> &Cloud_left, const boost::shared_ptr<Feature::myCloud> &Cloud_right);

    pcl::PolygonMesh
    greedyTriangulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                        const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);

};
