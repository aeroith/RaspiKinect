#include "filter.h"

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filter::removeNaNPoints(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {

    typedef pcl::PointCloud<pcl::PointXYZRGB> CloudType;
    CloudType::Ptr outputCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    CloudType::PointType p_nan;
    p_nan.x = std::numeric_limits<float>::quiet_NaN();
    p_nan.y = std::numeric_limits<float>::quiet_NaN();
    p_nan.z = std::numeric_limits<float>::quiet_NaN();
    cloud->push_back(p_nan);

    CloudType::PointType p_valid;
    p_valid.x = 1.0f;
    cloud->push_back(p_valid);

    std::cout << "Input size before NaN: " << cloud->points.size() << std::endl;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
    std::cout << "Output size after Nan: " << outputCloud->points.size()
              << std::endl;
    return outputCloud;
}
/* Remove (0,0,0) coordinates so surface normal estimation
*  can work successfully
*/
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filter::passThrough(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                    float min_val, float max_val) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    std::cout << "Input size before PassThrough filter: " << cloud->points.size()
              << std::endl;
    // Create the filtering object
    pcl::PassThrough<pcl::PointXYZRGB>::Ptr pass =
            boost::make_shared<pcl::PassThrough<pcl::PointXYZRGB>>();
    pass->setInputCloud(cloud);
    pass->setFilterFieldName("z");
    pass->setFilterLimits(min_val, max_val);
    pass->setKeepOrganized(true);
    pass->setFilterLimitsNegative(false);
    pass->filter(*cloud_filtered);

    std::cout << "Output size after PassThrough filter: "
              << cloud_filtered->points.size() << std::endl;
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filter::voxelGrid(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                  float leaf_size) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    std::cerr << "PointCloud before filtering: " << cloud->width * cloud->height
              << " data points (" << pcl::getFieldsList(*cloud) << ")."
              << std::endl;

    // Create the filtering object
    pcl::VoxelGrid<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setLeafSize(leaf_size, leaf_size, leaf_size);
    sor.filter(*cloud_filtered);

    std::cerr << "PointCloud after filtering: "
              << cloud_filtered->width * cloud_filtered->height
              << " data points (" << pcl::getFieldsList(*cloud_filtered) << ")."
              << std::endl;

    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Filter::statisticalOutlierRemoval(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, float mean,
        float std_dev) {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    std::cerr << "Cloud before filtering: " << std::endl;
    std::cerr << *cloud << std::endl;

    // Create the filtering object
    pcl::StatisticalOutlierRemoval<pcl::PointXYZRGB> sor;
    sor.setInputCloud(cloud);
    sor.setMeanK(mean);
    sor.setStddevMulThresh(std_dev);
    sor.filter(*cloud_filtered);

    std::cerr << "Cloud after filtering: " << std::endl;
    std::cerr << *cloud_filtered << std::endl;

    sor.setNegative(true);
    sor.filter(*cloud_filtered);
    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Filter::radiusOutlineRemoval(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &input, float radius,
        int neighbors) {

    pcl::RadiusOutlierRemoval<pcl::PointXYZRGB> out_rem;
    out_rem.setInputCloud(input);
    out_rem.setRadiusSearch(radius);
    out_rem.setMinNeighborsInRadius(neighbors);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    out_rem.filter(*cloud_filtered);

    return (cloud_filtered);
}

pcl::PointCloud<pcl::PointNormal>::Ptr
Filter::removeNaNNormals(const pcl::PointCloud<pcl::PointNormal>::Ptr &cloud) {

    typedef pcl::PointCloud<pcl::PointNormal> CloudType;
    CloudType::Ptr outputCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();

    CloudType::PointType p_nan;
    p_nan.x = std::numeric_limits<float>::quiet_NaN();
    p_nan.y = std::numeric_limits<float>::quiet_NaN();
    p_nan.z = std::numeric_limits<float>::quiet_NaN();
    cloud->push_back(p_nan);

    CloudType::PointType p_valid;
    p_valid.x = 1.0f;
    cloud->push_back(p_valid);

    std::cout << "InputNormal size before NaN: " << cloud->points.size()
              << std::endl;

    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *outputCloud, indices);
    std::cout << "OutputNormal size after Nan: " << outputCloud->points.size()
              << std::endl;
    return outputCloud;
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filter::matrixTransform(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                        char axis, float theta, float translation,
                        bool centroid) {

    Eigen::Affine3f transform = Eigen::Affine3f::Identity();
    Eigen::Matrix3f rotation;
    switch (axis) {
    case 'X':
        rotation = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());
        rotation = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY());
        break;
    case 'Y':
        rotation = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitY());
        break;
    case 'Z':
        rotation = Eigen::AngleAxisf(theta, Eigen::Vector3f::UnitZ());
        break;
    }

    transform.rotate(rotation);
    transform.translation() << -0.00146, -0.00572, 2.40577208;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    if (centroid) {
        Eigen::Vector4f centroid(Eigen::Vector4f::Zero());
        pcl::compute3DCentroid(*cloud, centroid);
        Eigen::Vector4f centroid_new(Eigen::Vector4f::Zero());
        centroid_new.head<3>() = rotation * centroid.head<3>();
        transform.translation() = centroid.head<3>() - centroid_new.head<3>();
    }
    pcl::transformPointCloud(*cloud, *transformed_cloud, transform);
    return (transformed_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filter::applyFilters(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                     float min_val, float max_val, float leaf_size,
                     float radius, int neighbors) {
    std::cerr << "Applying Filters..." << std::endl;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr temp_cloud;
    boost::shared_ptr<Filter> filter = boost::make_shared<Filter>();

    temp_cloud = filter->passThrough(cloud, min_val, max_val);
    temp_cloud = filter->bilateralFilter(temp_cloud);
    temp_cloud = filter->removeNaNPoints(temp_cloud);
    temp_cloud = filter->voxelGrid(temp_cloud, leaf_size);
    temp_cloud = filter->movingLeastSquaresUpsample(temp_cloud);
    temp_cloud = filter->radiusOutlineRemoval(temp_cloud, radius, neighbors);
    std::cerr << "Applied Filters." << std::endl;
    return (temp_cloud);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filter::medianFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr output =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    pcl::MedianFilter<pcl::PointXYZRGB> filter;
    filter.setInputCloud(cloud);
    filter.setWindowSize(2);
    filter.filter(*output);
    return (output);
}

// smooths the point cloud
pcl::PointCloud<pcl::PointNormal>::Ptr Filter::movingLeastSquaresSmooth(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    // Output has the PointNormal type in order to store the normals calculated by MLS
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree =
            boost::make_shared<pcl::search::KdTree<pcl::PointXYZRGB>>();

    pcl::PointCloud<pcl::PointNormal>::Ptr mls_points =
            boost::make_shared<pcl::PointCloud<pcl::PointNormal>>();
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointNormal> mls;
    mls.setComputeNormals(true);
    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setSearchMethod(tree);
    mls.setSearchRadius(0.05);
    // Reconstruct
    mls.process(*mls_points);
    return (mls_points);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr Filter::movingLeastSquaresUpsample(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    // Output has the PointNormal type in order to store the normals calculated by MLS
    std::cerr << "Applying MLS upsampling... " << std::endl;
    std::cout << "InputNormal size before MLS: " << cloud->points.size()
              << std::endl;
    // Init object (second point type is for the normals, even if unused)
    pcl::MovingLeastSquares<pcl::PointXYZRGB, pcl::PointXYZRGB> mls;
    // Set parameters
    mls.setInputCloud(cloud);
    mls.setPolynomialFit(true);
    mls.setPolynomialOrder(2);
    mls.setSearchRadius(0.01);
    mls.setUpsamplingMethod(
                pcl::MovingLeastSquares<pcl::PointXYZRGB,
                pcl::PointXYZRGB>::SAMPLE_LOCAL_PLANE);
    mls.setUpsamplingRadius(0.005);
    mls.setUpsamplingStepSize(0.003);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_smoothed =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();
    mls.process(*cloud_smoothed);
    std::cout << "InputNormal size before MLS: " << cloud_smoothed->points.size()
              << std::endl;
    return (cloud_smoothed);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Filter::bilateralFilter(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud) {
    pcl::FastBilateralFilter<pcl::PointXYZRGB> bilateral_filter;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    bilateral_filter.setInputCloud (cloud);
    bilateral_filter.setSigmaS (5);
    bilateral_filter.setSigmaR (5e-3);
    bilateral_filter.filter(*filtered_cloud);
    return(filtered_cloud);
}
