#include <pcl/io/ply_io.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <pcl/filters/uniform_sampling.h>
#include <pcl/console/time.h>   // TicToc

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

const double DISTANCE_THRESHOLD = 0.3;

const double FILTER_SIZE = 0.01;
bool FILTER = true;

int main(int argc, char **argv) {
    // The point clouds we will be using
    PointCloudT::Ptr cin1(new PointCloudT);  // Input point cloud1
    PointCloudT::Ptr cin2(new PointCloudT);  // Input point cloud1
    PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud

    // Checking program arguments
    if (argc < 3) {
        printf("Usage :\n");
        printf("\t\t%s file1.ply file2.ply\n", argv[0]);
        PCL_ERROR ("Provide two ply files.\n");
        return (-1);
    }

    // Define initial transformation matrix
    Eigen::Matrix4d transformation_matrix = Eigen::Matrix4d::Identity();

    if (argc > 4) {
        // If the user provided a matrix to transform point cloud 2
        if (argc != 20) {
            printf("Transformation matrix must have 16 elements. %d provided", argc - 4);
            PCL_ERROR("Invalid transform matrix.\n");
            return -1;
        }

        int matrixStart = 3;

        transformation_matrix << stod(argv[matrixStart + 0]), stod(argv[matrixStart + 1]), stod(
                argv[matrixStart + 2]), stod(argv[matrixStart + 3]),
                stod(argv[matrixStart + 4]), stod(argv[matrixStart + 5]), stod(argv[matrixStart + 6]), stod(
                argv[matrixStart + 7]),
                stod(argv[matrixStart + 8]), stod(argv[matrixStart + 9]), stod(argv[matrixStart + 10]), stod(
                argv[matrixStart + 11]),
                stod(argv[matrixStart + 12]), stod(argv[matrixStart + 13]), stod(argv[matrixStart + 14]), stod(
                argv[matrixStart + 15]);

        cout << "Using transformation matrix:\n" << transformation_matrix << endl;
    }

    pcl::console::TicToc time;
    time.tic();
    if (pcl::io::loadPLYFile(argv[1], *cin1) < 0) {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }
    cout << "Loaded file " << argv[1] << " (" << cin1->size() << " points) in " << time.toc() << " ms"
         << endl;

    time.tic();
    if (pcl::io::loadPLYFile(argv[2], *cin2) < 0) {
        PCL_ERROR ("Error loading cloud %s.\n", argv[2]);
        return (-1);
    }
    cout << "Loaded file " << argv[2] << " (" << cin2->size() << " points) in " << time.toc() << " ms"
         << endl;

    PointCloudT::Ptr cloud_in1(new PointCloudT);
    PointCloudT::Ptr cloud_in2(new PointCloudT);

    if (FILTER) {
        pcl::UniformSampling<PointT> sampler1;
        sampler1.setRadiusSearch(FILTER_SIZE);
        sampler1.setInputCloud(cin1);
        sampler1.filter(*cloud_in1);
        std::cerr << "Point cloud 1 after filtering: " << cloud_in1->size() << "." << endl;

        pcl::UniformSampling<PointT> sampler2;
        sampler2.setRadiusSearch(FILTER_SIZE);
        sampler2.setInputCloud(cin2);
        sampler2.filter(*cloud_in2);
        std::cerr << "Point cloud 2 after filtering: " << cloud_in2->size() << "." << endl;
    } else {
        *cloud_in1 = *cin1;
        *cloud_in2 = *cin2;
    }

    pcl::ModelCoefficients::Ptr coefficients1(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers1(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg1;
    // Optional
    seg1.setOptimizeCoefficients(true);
    // Mandatory
    seg1.setModelType(pcl::SACMODEL_PLANE);
    seg1.setMethodType(pcl::SAC_RANSAC);
    seg1.setDistanceThreshold(DISTANCE_THRESHOLD);
    seg1.setInputCloud(cin1);
    seg1.segment(*inliers1, *coefficients1);


    pcl::ModelCoefficients::Ptr coefficients2(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers2(new pcl::PointIndices);
    // Create the segmentation object
    pcl::SACSegmentation<PointT> seg2;
    // Optional
    seg2.setOptimizeCoefficients(true);
    // Mandatory
    seg2.setModelType(pcl::SACMODEL_PLANE);
    seg2.setMethodType(pcl::SAC_RANSAC);
    seg2.setDistanceThreshold(DISTANCE_THRESHOLD);
    seg2.setInputCloud(cin2);
    seg2.segment(*inliers2, *coefficients2);

    if (inliers1->indices.size() == 0) {
        PCL_ERROR ("Could not estimate a planar model for dataset 1.");
        return (-1);
    }

    if (inliers2->indices.size() == 0) {
        PCL_ERROR ("Could not estimate a planar model for dataset 2.");
        return (-1);
    }

    std::cerr << "Model 1 coefficients: " << coefficients1->values[0] << " "
              << coefficients1->values[1] << " "
              << coefficients1->values[2] << " "
              << coefficients1->values[3] << std::endl;

    /*
    std::cerr << "Model 1 inliers: " << inliers1->indices.size () << std::endl;
    for (size_t i = 0; i < inliers1->indices.size (); ++i) {
        std::cerr << inliers1->indices[i] << "    " << cin1->points[inliers1->indices[i]].x << " "
                  << cin1->points[inliers1->indices[i]].y << " "
                  << cin1->points[inliers1->indices[i]].z << std::endl;
    }
    */

    std::cerr << "Model 2 coefficients: " << coefficients2->values[0] << " "
              << coefficients2->values[1] << " "
              << coefficients2->values[2] << " "
              << coefficients2->values[3] << std::endl;

    /*
    std::cerr << "Model 2 inliers: " << inliers2->indices.size () << std::endl;
    for (size_t i = 0; i < inliers2->indices.size (); ++i) {
        std::cerr << inliers2->indices[i] << "    " << cin2->points[inliers2->indices[i]].x << " "
                  << cin2->points[inliers2->indices[i]].y << " "
                  << cin2->points[inliers2->indices[i]].z << std::endl;
    }
    */

    return (0);
}