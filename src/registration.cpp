#include "registration.h"

Eigen::Matrix4f Registration::computeInitialAlignment(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &source_descriptors,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &target_descriptors,
        float min_sample_distance, float max_correspondence_distance,
        int nr_iterations) {

    pcl::console::print_highlight("starting initial alignment...\n");

    pcl::SampleConsensusInitialAlignment<pcl::PointXYZRGB, pcl::PointXYZRGB,
            pcl::FPFHSignature33>::Ptr
            sac_ia(new pcl::SampleConsensusInitialAlignment<
                   pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::FPFHSignature33>());

    sac_ia->setMinSampleDistance(min_sample_distance);
    sac_ia->setMaxCorrespondenceDistance(max_correspondence_distance);
    sac_ia->setMaximumIterations(nr_iterations);

    sac_ia->setInputSource(source_points);
    sac_ia->setSourceFeatures(source_descriptors);

    sac_ia->setInputTarget(target_points);
    sac_ia->setTargetFeatures(target_descriptors);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr outCloud =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    sac_ia->align(*outCloud);

    return (sac_ia->getFinalTransformation());
}

Eigen::Matrix4f Registration::refineAlignment(
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &source_points,
        const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &target_points,
        const Eigen::Matrix4f &initial_alignment, float max_correspondence_distance,
        float outlier_rejection_threshold, float transformation_epsilon,
        int max_iterations) {

    pcl::console::print_highlight("starting refined alignment...\n");

    pcl::PointCloud<pcl::PointXYZRGB> register_output;

    pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>::Ptr icp(
                new pcl::IterativeClosestPoint<pcl::PointXYZRGB, pcl::PointXYZRGB>());

    // specify icp parameters (needed)
    icp->setMaxCorrespondenceDistance(max_correspondence_distance);
    icp->setRANSACOutlierRejectionThreshold(outlier_rejection_threshold);
    icp->setTransformationEpsilon(transformation_epsilon);
    icp->setMaximumIterations(max_iterations);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sourcePointsTransformed =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    pcl::transformPointCloud(*source_points, *sourcePointsTransformed,
                             initial_alignment);

    icp->setInputSource(sourcePointsTransformed);
    icp->setInputTarget(target_points);

    icp->align(register_output);

    return (icp->getFinalTransformation() * initial_alignment);
}
pcl::PolygonMesh Registration::poissonReconstruct(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud,
        const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals) {
    using namespace pcl;
    console::print_highlight("Starting poisson reconstruction...\n");
    PointCloud<PointNormal>::Ptr cloud_smoothed_normals(
                new PointCloud<PointNormal>());
    concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);
    Poisson<PointNormal> poisson;
    poisson.setDepth(9);
    poisson.setInputCloud(cloud_smoothed_normals);
    PolygonMesh mesh;
    poisson.reconstruct(mesh);
    return (mesh);
}
pcl::PointCloud<pcl::PointXYZRGB>::Ptr
Registration::applyICP(const boost::shared_ptr<Feature::myCloud> &Cloud_left,
                       const boost::shared_ptr<Feature::myCloud> &Cloud_right) {

    double min_sample_dist = 1e-8;
    double max_correspondence_dist = 0.01f;
    double nr_iters = 10000;
    Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_out =
            boost::make_shared<pcl::PointCloud<pcl::PointXYZRGB>>();

    boost::shared_ptr<Registration> registration =
            boost::make_shared<Registration>();
    // find the transform that roughly aligns the points
    transform = registration->computeInitialAlignment(
                Cloud_left->keypoints, Cloud_left->local_descriptor,
                Cloud_right->keypoints, Cloud_right->local_descriptor, min_sample_dist,
                max_correspondence_dist, nr_iters);

    pcl::console::print_info("computed initial alignment!\n");
    float max_correspondence_distance =
            0.03f;
    float outlier_rejection_threshold =
            0.80f;
    float transformation_epsilon = 1e-8;
    int max_iterations = 1000;

    transform = registration->refineAlignment(
                Cloud_left->points, Cloud_right->points, transform,
                max_correspondence_distance, outlier_rejection_threshold,
                transformation_epsilon, max_iterations);

    pcl::console::print_info("refined alignment!\n");

    pcl::transformPointCloud(*Cloud_left->points, *cloud_out, transform);
    (*cloud_out) += (*Cloud_right->points);
    return (cloud_out);
}
pcl::PolygonMesh Registration::greedyTriangulation(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud,
                                                   const pcl::PointCloud<pcl::Normal>::Ptr &cloud_normals){
    using namespace pcl;
    console::print_highlight("Starting greedy triangulation...\n");
    PointCloud<PointXYZRGBNormal>::Ptr cloud_smoothed_normals(
                new PointCloud<PointXYZRGBNormal>());
    concatenateFields(*cloud, *cloud_normals, *cloud_smoothed_normals);
    pcl::search::KdTree<pcl::PointXYZRGBNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZRGBNormal>);
    tree->setInputCloud (cloud_smoothed_normals);
    pcl::GreedyProjectionTriangulation<pcl::PointXYZRGBNormal> gp3;
    pcl::PolygonMesh triangles;
    gp3.setSearchRadius (0.005);

    // Set typical values for the parameters
    gp3.setMu (2.5);
    gp3.setMaximumNearestNeighbors (1000);
    gp3.setMaximumSurfaceAngle(M_PI/4); // 45 degrees
    gp3.setMinimumAngle(M_PI/18); // 10 degrees
    gp3.setMaximumAngle(2*M_PI/3); // 120 degrees
    gp3.setNormalConsistency(false);
    gp3.setInputCloud (cloud_smoothed_normals);
    gp3.setSearchMethod (tree);
    gp3.reconstruct (triangles);
    return(triangles);
}
