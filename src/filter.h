#pragma once

#include <pcl/common/transforms.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/median_filter.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/radius_outlier_removal.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/surface/mls.h>
#include <pcl/filters/fast_bilateral.h>
class Filter {
public:
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    removeNaNPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    passThrough(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                float min_val, float max_val);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    voxelGrid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
              float leaf_size);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    statisticalOutlierRemoval(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input,
                              float mean, float std_dev);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    radiusOutlineRemoval(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                         float radius, int neighbors);
    pcl::PointCloud<pcl::PointNormal>::Ptr
    removeNaNNormals(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    matrixTransform(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                    char axis, float theta, float translation, bool centroid);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    medianFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) ;

    pcl::PointCloud<pcl::PointNormal>::Ptr
    movingLeastSquaresSmooth(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr movingLeastSquaresUpsample(
            const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    applyFilters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                 float min_val, float max_val, float leaf_size, float radius,
                 int neighbors);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr
    bilateralFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) ;

};
