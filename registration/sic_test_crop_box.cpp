#include <iostream>
#include <string>

#include <unistd.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/uniform_sampling.h>
#include <pcl/filters/crop_box.h>
#include <pcl/common/io.h>
#include <pcl/registration/gicp.h>

using namespace std;

typedef pcl::PointXYZ PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

bool next_iteration = false;

const double FILTER_SIZE = 0.01;
bool FILTER = true;

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

    // The user provides a matrix to transform point cloud 2

    int matrixStart = 4;

    transformation_matrix << stof(argv[matrixStart + 0]), stof(argv[matrixStart + 1]), stof(
            argv[matrixStart + 2]), stof(argv[matrixStart + 3]),
            stof(argv[matrixStart + 4]), stof(argv[matrixStart + 5]), stof(argv[matrixStart + 6]), stof(
            argv[matrixStart + 7]),
            stof(argv[matrixStart + 8]), stof(argv[matrixStart + 9]), stof(argv[matrixStart + 10]), stof(
            argv[matrixStart + 11]),
            stof(argv[matrixStart + 12]), stof(argv[matrixStart + 13]), stof(argv[matrixStart + 14]), stof(
            argv[matrixStart + 15]);

    cout << "Using transformation matrix:\n" << transformation_matrix << endl;

    Eigen::Vector4f points1[3];
    Eigen::Vector4f points2[3];

    int start = 20;
    for (int i = 0; i < 3; i++) {
        points1[i] << stof(argv[start + 3 * i + 0]),
                stof(argv[start + 3 * i + 1]),
                stof(argv[start + 3 * i + 2]), 1.0f;
    }
    start = 29;
    for (int i = 0; i < 3; i++) {
        points2[i] << stof(argv[start + 3 * i + 0]),
                stof(argv[start + 3 * i + 1]),
                stof(argv[start + 3 * i + 2]), 1.0f;
    }

    cout << "\nUsing points1:" << endl;
    for (int i = 0; i < 3; i++) {
        cout << "[" << points1[i] << "]" << endl;
    }
    cout << "\nUsing points2:" << endl;
    for (int i = 0; i < 3; i++) {
        cout << "[" << points2[i] << "]" << endl;
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

    PointCloudT::Ptr filter_out1(new PointCloudT);
    PointCloudT::Ptr filter_out2(new PointCloudT);

    if (FILTER) {
        pcl::UniformSampling<PointT> sampler1;
        sampler1.setRadiusSearch(FILTER_SIZE);
        sampler1.setInputCloud(cin1);
        sampler1.filter(*filter_out1);
        std::cerr << "Point cloud 1 size after filtering: " << filter_out1->size() << endl;

        pcl::UniformSampling<PointT> sampler2;
        sampler2.setRadiusSearch(FILTER_SIZE);
        sampler2.setInputCloud(cin2);
        sampler2.filter(*filter_out2);
        std::cerr << "Point cloud 2 size after filtering: " << filter_out2->size() << endl;
    } else {
        *filter_out1 = *cin1;
        *filter_out2 = *cin2;
    }

    Eigen::Vector4f min1 = points1[0];
    Eigen::Vector4f max1 = points1[0];
    Eigen::Vector4f min2 = points2[0];
    Eigen::Vector4f max2 = points2[0];

    min1(3) = 1.0f;
    max1(3) = 1.0f;
    min2(3) = 1.0f;
    max2(3) = 1.0f;

    for (int i = 0; i < 3; i++) {
        for (int j = 0; j < 3; j++) {
            if (points1[i](j) < min1(j)) {
                min1(j) = points1[i](j);
            }
            if (points1[i](j) > max1(j)) {
                max1(j) = points1[i](j);
            }
            if (points2[i](j) < min2(j)) {
                min2(j) = points2[i](j);
            }
            if (points2[i](j) > max2(j)) {
                max2(j) = points2[i](j);
            }
        }
    }

    float box_expand = 0.5f;
    for (int i = 0; i < 3; i++) {
        min1(i) -= box_expand;
        max1(i) += box_expand;
        min2(i) -= box_expand;
        max2(i) += box_expand;
    }

    cout << "\nmin1:\n[" << min1 << "]" << endl;
    cout << "max1:\n[" << max1 << "]" << endl;
    cout << "min2:\n[" << min2 << "]" << endl;
    cout << "max2:\n[" << max2 << "]" << endl;

    PointCloudT::Ptr cloud_in1(new PointCloudT);
    PointCloudT::Ptr cloud_in2(new PointCloudT);

    for (PointCloudT::iterator it = filter_out1->begin(); it != filter_out1->end(); it++) {
        if (it->x > min1(0) && it->y > min1(1) && it->z > min1(2) &&
            it->x < max1(0) && it->y < max1(1) && it->z < max1(2)) {
            cloud_in1->push_back(PointT(it->x, it->y, it->z));
        }
    }
    std::cerr << "Point cloud 1 size after crop box: " << cloud_in1->size() << endl;

    for (PointCloudT::iterator it = filter_out2->begin(); it != filter_out2->end(); it++) {
        if (it->x > min2(0) && it->y > min2(1) && it->z > min2(2) &&
            it->x < max2(0) && it->y < max2(1) && it->z < max2(2)) {
            cloud_in2->push_back(PointT(it->x, it->y, it->z));
        }
    }
    std::cerr << "Point cloud 2 size after crop box: " << cloud_in2->size() << endl;

    *cloud_tr = *cloud_in2;

    // Transform second point cloud with provided matrix
    pcl::transformPointCloud(*cloud_tr, *cloud_in2, transformation_matrix);

    // The Iterative Closest Point algorithm
    time.tic();
    pcl::GeneralizedIterativeClosestPoint<PointT, PointT> icp;
    icp.setMaximumIterations(iterations);
    icp.setInputSource(cloud_in2);
    icp.setInputTarget(cloud_in1);

    cout << "Starting ICP" << endl;

    PointCloudT::Ptr final_result(new PointCloudT);

    icp.align(*final_result);
    icp.setMaximumIterations(1);  // We set this variable to 1 for the next time we will call .align () function
    cout << "Applied " << iterations << " ICP iteration(s) in " << (time.toc() / 1000.0) << " seconds"
         << endl;

    /*
    if (icp.hasConverged()) {
        cout << "\nICP has converged, score is " << icp.getFitnessScore() << endl;
        cout << "\nICP transformation " << iterations << " : cloud_icp -> cloud_in" << endl;
        transformation_matrix = icp.getFinalTransformation().cast<double>();
        cout << transformation_matrix << endl;
    } else {
        PCL_ERROR ("\nICP has not converged.\n");
        return (-1);
    }
     */
    transformation_matrix = icp.getFinalTransformation().cast<double>();

    cout << transformation_matrix << endl;

    return 0;

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

        viewer.addPointCloud(final_result, cloud_icp_color_h, "cloud_icp_v2", v2);

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
            icp.align(*final_result);
            std::cout << "Applied 1 ICP iteration in " << time.toc() << " ms" << std::endl;

            if (icp.hasConverged()) {
                printf("\033[11A");  // Go up 11 lines in terminal output.
                printf("\nICP has converged, score is %+.0e\n", icp.getFitnessScore());
                std::cout << "\nICP transformation " << ++iterations << " : cloud_in2 -> cloud_in1" << std::endl;
                transformation_matrix *= icp.getFinalTransformation().cast<double>();  // WARNING /!\ This is not accurate! For "educational" purpose only!
                cout << transformation_matrix
                     << endl;  // Print the transformation between original pose and current pose

                ss.str("");
                ss << iterations;
                std::string iterations_cnt = "ICP iterations = " + ss.str();
                viewer.updateText(iterations_cnt, 10, 60, 16, txt_gray_lvl, txt_gray_lvl, txt_gray_lvl,
                                  "iterations_cnt");
                viewer.updatePointCloud(final_result, cloud_icp_color_h, "cloud_icp_v2");
            } else {
                PCL_ERROR ("\nICP has not converged.\n");
                return (-1);
            }
        }
        next_iteration = false;
    }

    return (0);
}
