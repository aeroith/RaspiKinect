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
 /** Use SampleConsensusInitialAlignment to find a rough alignment from the source cloud to the target cloud by finding
 * correspondences between two sets of local features
 *  @param[in] source_points
      The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   @param[in] source_descriptors
      The local descriptors for each source point
 *   @param[in] target_points
      The "target" points, i.e., the points to which the source point cloud will be aligned
 *   @param[in] target_descriptors
      The local descriptors for each target point
 *  @param[in] min_sample_distance
      The minimum distance between any two random samples
 *  @param[in] max_correspondence_distance
      The
 *  @param[in] nr_iterations
      The number of RANSAC iterations to perform
 * Return: A transformation matrix that will roughly align the points in source to the points in target
 */
    Eigen::Matrix4f computeInitialAlignment(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
            const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &source_descriptors,
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
            const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &target_descriptors,
            float min_sample_distance, float max_correspondence_distance,
            int nr_iterations);

/** Use IterativeClosestPoint to find a precise alignment from the source cloud to the target cloud,
 * starting with an intial guess
 *   @param[in] source_points
      The "source" points, i.e., the points that must be transformed to align with the target point cloud
 *   @param[in] target_points
      The "target" points, i.e., the points to which the source point cloud will be aligned
 *   @param[in] initial_alignment
      An initial estimate of the transformation matrix that aligns the source points to the target points
 *   @param[in] max_correspondence_distance
      A threshold on the distance between any two corresponding points.  Any corresponding points that are further
      apart than this threshold will be ignored when computing the source-to-target transformation
 *   @param[in] outlier_rejection_threshold
      A threshold used to define outliers during RANSAC outlier rejection
 *   @param[in] transformation_epsilon
      The smallest iterative transformation allowed before the algorithm is considered to have converged
 *   @param[in] max_iterations
      The maximum number of ICP iterations to perform
 * Return: A transformation matrix that will precisely align the points in source to the points in target
 */
    Eigen::Matrix4f
    refineAlignment(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
                    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
                    const Eigen::Matrix4f &initial_alignment,
                    float max_correspondence_distance,
                    float outlier_rejection_threshold,
                    float transformation_epsilon, int max_iterations);

/**
* Apply poisson reconstruction to the point clouds. Does not yield very good results
* for kinect output.
* @param[in] cloud input point cloud
* @param[out] cloud_normals point cloud normals
*/
    pcl::PolygonMesh
    poissonReconstruct(const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
                       const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);

/**
* Apply ICP using the computeInitialAlignment() and refineAlignment()
* functions and obtain combined clouds. Returns left hand side point 
* cloud as a combination of both.
* @param[in] Cloud_left left hand side myCloud struct 
* @param[in] Cloud_right right hand side myCloud struct 
*/
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    applyICP(const boost::shared_ptr<Feature::myCloud> &Cloud_left, const boost::shared_ptr<Feature::myCloud> &Cloud_right);

/**
* Apply a greedy surface triangulation algorithm on a PointCloud with normals, 
* to obtain a triangle mesh based on projections of the local neighborhoods.
* @param[in] cloud input point cloud
* @param[in] cloud_normals point cloud normals
*/
    pcl::PolygonMesh
    greedyTriangulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                        const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals);

};
