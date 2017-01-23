#include "feature.h"

// Can be further optimized by using OpenMP
pcl::PointCloud<pcl::Normal>::Ptr Feature::estimateSurfaceNormals(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float radius) {
    std::cerr << "Starting surface estimation... ";
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;

    // Create an empty kdtree representation, and pass it to the normal estimation
    // object.
    // Its content will be filled inside the object, based on the given input
    // dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree =
            boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals =
            boost::make_shared<pcl::PointCloud<pcl::Normal>>();

    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);

    std::cerr << "DONE!" << std::endl;

    // cloud_normals->points.size () should have the same size as the input
    // cloud->points.size ()*
    return (cloud_normals);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Feature::getSiftKeypoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                          const float min_scale, const int n_octaves,
                          const int n_scales_per_octave,
                          const float min_contrast) {
    std::cerr << "Getting Pointcloud Keypoints... ";

    pcl::SIFTKeypoint<pcl::PointXYZRGB, pcl::PointWithScale> sift;
    pcl::PointCloud<pcl::PointWithScale> result;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree =
            boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();

    sift.setSearchMethod(tree);
    sift.setScales(min_scale, n_octaves, n_scales_per_octave);
    sift.setMinimumContrast(min_contrast);
    sift.setInputCloud(cloud);
    sift.compute(result);
    std::cerr << "DONE! ";
    std::cerr << "point size: " << result.size() << std::endl;
    // Copying the pointwithscale to pointxyz so as visualize the cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_temp =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    copyPointCloud(result, *cloud_temp);

    return (cloud_temp);
}

// can be further optimized by OpenMP
pcl::PointCloud<pcl::FPFHSignature33>::Ptr Feature::getLocalDescriptor(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &keypoints, float radius) {
    std::cerr << "Getting Local Descriptors... ";

    // Create the FPFH estimation class, and pass the input
    // dataset+normals to it
    pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal,
            pcl::FPFHSignature33>::Ptr fpfh =
            boost::make_shared<pcl::FPFHEstimationOMP<pcl::PointXYZRGB, pcl::Normal,
            pcl::FPFHSignature33>>();
    fpfh->setNumberOfThreads(8);
    fpfh->setInputCloud(keypoints);
    fpfh->setInputNormals(normals);
    // alternatively, if cloud is of type PointNormal, do fpfh.setInputNormals
    // (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation
    // object.
    // Its content will be filled inside the object, based on the given input
    // dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree =
            boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();

    fpfh->setSearchMethod(tree);
    fpfh->setSearchSurface(cloud);

    // Output datasets
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr fpfhs =
            boost::make_shared<pcl::PointCloud<pcl::FPFHSignature33>>();

    // Use all neighbors in a sphere of radius
    // IMPORTANT: the radius used here has to be larger than the radius used
    // to
    // estimate the surface normals!!!
    fpfh->setRadiusSearch(radius);

    // Compute the features
    fpfh->compute(*fpfhs);

    std::cerr << "DONE!" << std::endl;
    return (fpfhs);
}
// This function will estimate a set of VFH features for all the points in the
// input dataset.
pcl::PointCloud<pcl::VFHSignature308>::Ptr Feature::getGlobalDescriptor(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &normals) {

    std::cerr << "Getting Global Descriptors... ";
    // Create the VFH estimation class, and pass the input
    // dataset+normals to it
    pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::VFHSignature308>::Ptr
            vfh = boost::make_shared<pcl::VFHEstimation<pcl::PointXYZRGB, pcl::Normal,
            pcl::VFHSignature308>>();
    vfh->setInputCloud(cloud);
    vfh->setInputNormals(normals);
    // alternatively, if cloud is of type PointNormal, do vfh->setInputNormals
    // (cloud);

    // Create an empty kdtree representation, and pass it to the FPFH estimation
    // object.
    // Its content will be filled inside the object, based on the given input
    // dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree =
            boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();
    vfh->setSearchMethod(tree);

    // Output datasets
    pcl::PointCloud<pcl::VFHSignature308>::Ptr vfhs =
            boost::make_shared<pcl::PointCloud<pcl::VFHSignature308>>();

    // Compute the features
    vfh->compute(*vfhs);

    std::cerr << "DONE!" << std::endl;
    // vfhs->points.size () should be of size 1*
    return (vfhs);
}
boost::shared_ptr<Feature::myCloud>
Feature::applyFeatures(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {

    pcl::console::print_info("Applying Pointcloud features...\n");
    boost::shared_ptr<myCloud> feature = boost::make_shared<myCloud>();

    feature->points = cloud;
    // feature->normals = estimateSurfaceNormalsOMP(cloud, 0.01);
    feature->keypoints = getSiftKeypoints(cloud, 0.005, 10, 8, 1.5);
    feature->local_descriptor =
            getLocalDescriptor(cloud, feature->normals, feature->keypoints, 0.1);
    feature->global_descriptor = getGlobalDescriptor(cloud, feature->normals);

    return feature;
}

pcl::PointCloud<pcl::Normal>::Ptr Feature::estimateSurfaceNormalsOMP(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float radius) {
    std::cerr << "Starting surface estimation with OMP... ";
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> ne;

    // Create an empty kdtree representation, and pass it to the normal estimation
    // object.
    // Its content will be filled inside the object, based on the given input
    // dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree =
            boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();

    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals =
            boost::make_shared<pcl::PointCloud<pcl::Normal>>();
    ne.setNumberOfThreads(8);
    ne.setSearchMethod(tree);
    ne.setRadiusSearch(radius);
    ne.setInputCloud(cloud);
    ne.compute(*cloud_normals);
    Eigen::Vector4f centroid;
    compute3DCentroid(*cloud, centroid);
    ne.setViewPoint(centroid[0], centroid[1], centroid[2]);
    for (size_t i = 0; i < cloud_normals->size(); ++i) {
        cloud_normals->points[i].normal_x *= -1;
        cloud_normals->points[i].normal_y *= -1;
        cloud_normals->points[i].normal_z *= -1;
    }
    std::cerr << "DONE!" << std::endl;
    return (cloud_normals);
}
