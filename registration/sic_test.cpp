#include <iostream>
#include <string>

#include <unistd.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

int main(int argc,
         char *argv[]) {
    // The point clouds we will be using
    PointCloudT::Ptr cloud_in(new PointCloudT);  // Original point cloud
    PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud
    PointCloudT::Ptr cloud_icp(new PointCloudT);  // ICP output point cloud

    // Checking program arguments
    if (argc < 3) {
        printf("Usage :\n");
        printf("\t\t%s file1.ply file2.ply [number_of_ICP_iterations]\n", argv[0]);
        PCL_ERROR ("Provide two ply files.\n");
        return (-1);
    }

    int iterations = 1;  // Default number of ICP iterations
    if (argc > 3) {
        // If the user passed the number of iteration as an argument
        iterations = atoi(argv[3]);
        if (iterations < 1) {
            PCL_ERROR ("Number of initial iterations must be >= 1\n");
            return (-1);
        }
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

        int matrixStart = 4;

        transformation_matrix << stod(argv[matrixStart + 0]), stod(argv[matrixStart + 1]), stod(argv[matrixStart + 2]), stod(argv[matrixStart + 3]),
                stod(argv[matrixStart + 4]), stod(argv[matrixStart + 5]), stod(argv[matrixStart + 6]), stod(argv[matrixStart + 7]),
                stod(argv[matrixStart + 8]), stod(argv[matrixStart + 9]), stod(argv[matrixStart + 10]), stod(argv[matrixStart + 11]),
                stod(argv[matrixStart + 12]), stod(argv[matrixStart + 13]), stod(argv[matrixStart + 14]), stod(argv[matrixStart + 15]);

        cout << "Using transformation matrix:\n" << transformation_matrix << endl;
    }

    pcl::console::TicToc time;
    time.tic();
    if (pcl::io::loadPLYFile(argv[1], *cloud_in) < 0) {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }
    cout << "Loaded file " << argv[1] << " (" << cloud_in->size() << " points) in " << time.toc() << " ms"
              << endl;

    time.tic();
    if (pcl::io::loadPLYFile(argv[2], *cloud_icp) < 0) {
        PCL_ERROR ("Error loading cloud %s.\n", argv[2]);
        return (-1);
    }
    cout << "Loaded file " << argv[2] << " (" << cloud_icp->size() << " points) in " << time.toc() << " ms"
              << endl;

    // Transform second point cloud with provided matrix
    *cloud_tr = *cloud_icp;  // We backup cloud_icp into cloud_tr for later use
    pcl::transformPointCloud(*cloud_tr, *cloud_icp, transformation_matrix);
    *cloud_tr = *cloud_icp;

    // The Iterative Closest Point algorithm
    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_icp);
    icp.setInputTarget(cloud_in);

    cout << "Starting ICP" << endl;

    icp.align(*cloud_icp);
    icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
    cout << "Applied " << iterations << " ICP iteration(s) in " << (time.toc() / 1000.0) << " seconds"
              << endl;

    if (icp.hasConverged()) {
        cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
        cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        cout << transformation_matrix << endl;
    } else {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    return (0);
}
