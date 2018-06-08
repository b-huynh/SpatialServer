#include <iostream>
#include <string>

#include <unistd.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/uniform_sampling.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

const double FILTER_SIZE = 0.01;
const bool FILTER = true;

void keyboardEventOccurred(const pcl::visualization::KeyboardEvent &event,
                           void *nothing) {
    if (event.getKeySym() == "space" && event.keyDown()) {
        cout << "Starting next iteration" << endl;
        next_iteration = true;
    } else {
        cout << event.getKeySym() << endl;
    }
}

int main(int argc,
         char *argv[]) {
    // The point clouds we will be using
    PointCloudT::Ptr cin1(new PointCloudT);  // Input point cloud1
    PointCloudT::Ptr cin2(new PointCloudT);  // Input point cloud1
    PointCloudT::Ptr cloud_tr(new PointCloudT);  // Transformed point cloud

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
        pcl::UniformSampling<PointT> sampler;
        sampler.setRadiusSearch(FILTER_SIZE);

        sampler.setInputCloud(cin1);
        sampler.filter(*cloud_in1);
        std::cerr << "Point cloud 1 after filtering: " << cloud_in1->size() << "." << endl;

        sampler.setInputCloud(cin2);
        sampler.filter(*cloud_in2);
        std::cerr << "Point cloud 2 after filtering: " << cloud_in2->size() << "." << endl;
    } else {
        cloud_in1 = cin1;
        cloud_in2 = cin2;
    }

    // Transform second point cloud with provided matrix
    *cloud_tr = *cloud_in2;
    pcl::transformPointCloud(*cloud_tr, *cloud_in2, transformation_matrix);
    *cloud_tr = *cloud_in2;

    // The Iterative Closest Point algorithm
    time.tic();
    pcl::IterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_in2);
    icp.setInputTarget(cloud_in1);

    cout << "Starting ICP" << endl;

    icp.align(*cloud_in2);
    icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
    cout << "Applied " << iterations << " ICP iteration(s) in " << (time.toc() / 1000.0) << " seconds"
         << endl;

    if (icp.hasConverged()) {
        cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
        cout << "\nICP transformation " << iterations << " : cloud_in2 -> cloud_in1" << endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        cout << transformation_matrix << endl;
    } else {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }

    pcl::visualization::PCLVisualizer viewer("ICP demo");
    // The color we will be using
    float bckgr_gray_level = 0.0;  // Black
    float txt_gray_lvl = 1.0 - bckgr_gray_level;
    // ICP aligned point cloud is red
    pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_icp_color_h(cloud_in2, 180, 20, 20);
    std::stringstream ss;
    {
        // Create two vertically separated viewports
        int v1(0);
        int v2(1);
        viewer.createViewPort(0.0, 0.0, 0.5, 1.0, v1);
        viewer.createViewPort(0.5, 0.0, 1.0, 1.0, v2);

        // Original point cloud is white
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_in_color_h(cloud_in1, (int) 255 * txt_gray_lvl,
                                                                                  (int) 255 * txt_gray_lvl,
                                                                                  (int) 255 * txt_gray_lvl);
        viewer.addPointCloud(cloud_in1, cloud_in_color_h, "cloud_in_v1", v1);
        viewer.addPointCloud(cloud_in1, cloud_in_color_h, "cloud_in_v2", v2);

        // Transformed point cloud is green
        pcl::visualization::PointCloudColorHandlerCustom<PointT> cloud_tr_color_h(cloud_tr, 20, 180, 20);
        viewer.addPointCloud(cloud_tr, cloud_tr_color_h, "cloud_tr_v1", v1);

        viewer.addPointCloud(cloud_in2, cloud_icp_color_h, "cloud_icp_v2", v2);

        // Adding text descriptions in each viewport
        viewer.addText("White: Original point cloud 1\nGreen: Original point cloud 2", 10, 15, 16, txt_gray_lvl,
                       txt_gray_lvl, txt_gray_lvl, "icp_info_1", v1);
        viewer.addText("White: Original point cloud 1\nRed: ICP aligned point cloud 2", 10, 15, 16, txt_gray_lvl,
                       txt_gray_lvl,
                       txt_gray_lvl, "icp_info_2", v2);

        ss << iterations;
        std::string iterations_cnt = "ICP iterations = " + ss.str();
        viewer.addText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl, "iterations_cnt", v2);

        // Set background color
        viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v1);
        viewer.setBackgroundColor(bckgr_gray_level, bckgr_gray_level, bckgr_gray_level, v2);

        // Set camera position and orientation
        viewer.setCameraPosition(-3.68332, 2.94092, 5.71266, 0.289847, 0.921947, -0.256907, 0);
        viewer.setSize(1280, 1024);  // Visualiser window size

        // Register keyboard callback :
        viewer.registerKeyboardCallback(&keyboardEventOccurred, (void *) NULL);
    }

    // Display the visualiser
    while (!viewer.wasStopped()) {
        viewer.spinOnce();

        // The user pressed "space" :
        if (next_iteration) {
            // The Iterative Closest Point algorithm
            time.tic();
            icp.align(*cloud_in2);
            std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

            if (icp.hasConverged()) {
                printf("\033[11A");  // Go up 11 lines in terminal output.
                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_in2 -> cloud_in1" << std::endl;
                transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                cout << transformation_matrix << endl;  // Print the transformation between original pose and current pose

                ss.str("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str();
                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,
                                  "iterations_cnt");
                viewer.updatePointCloud(cloud_in2, cloud_icp_color_h, "cloud_icp_v2");
            } else {
                PCL_ERROR ("\nICP has not converged.\n");
                return (-1);
            }
        }
        next_iteration = false;
    }
    return (0);
}
