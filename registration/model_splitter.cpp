#include <iostream>
#include <string>

#include <unistd.h>

#include <pcl/io/ply_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/console/time.h>   // TicToc
#include <pcl/filters/uniform_sampling.h>
#include <pcl/common/io.h>
#include <pcl/registration/gicp.h>
#include <pcl/registration/icp.h>
#include <random>
#include <chrono>

using namespace std;

typedef pcl::PointXYZRGBNormal PointT;
typedef pcl::PointCloud<PointT> PointCloudT;

int main(int argc,
         char *argv[]) {
    // The point clouds we will be using
    PointCloudT::Ptr cin1(new PointCloudT);  // Input point cloud1

    // Checking program arguments
    if (argc < 2) {
        printf("Usage :\n");
        printf("\t\t%s file1.ply <reference points>\n", argv[0]);
        return (-1);
    }

    pcl::console::TicToc time;
    time.tic();
    if (pcl::io::loadPLYFile(argv[1], *cin1) < 0) {
        PCL_ERROR ("Error loading cloud %s.\n", argv[1]);
        return (-1);
    }
    cout << "Loaded file " << argv[1] << " (" << cin1->size() << " points) in " << time.toc() << " ms"
         << endl;

    Eigen::Vector3f points1[3];
    int start = 2;
    for (int i = 0; i < 3; i++) {
        points1[i] << stof(argv[start + 3 * i + 0]),
                stof(argv[start + 3 * i + 1]),
                stof(argv[start + 3 * i + 2]);
    }

    Eigen::Vector3f side1 = points1[1] - points1[0];
    Eigen::Vector3f side2 = points1[2] - points1[0];

    Eigen::Vector3f norm = side1.cross(side2);

    Eigen::Hyperplane<float, 3> plane(norm, points1[0]);

    PointCloudT::Ptr cout1(new PointCloudT);
    PointCloudT::Ptr cout2(new PointCloudT);

    unsigned seed = chrono::system_clock::now().time_since_epoch().count();
    default_random_engine generator (seed);
    uniform_real_distribution<double> distribution (0.0,1.0);

    int count = 0;
    for (PointCloudT::iterator it = cin1->begin(); it != cin1->end(); it++) {
        Eigen::Vector3f point;
        point << it->x, it->y, it->z;
        float dist = plane.signedDistance(point);

        dist *= 40;

        float limit = 1.0f / (2.0f + (dist * dist * dist * dist));
        float rand = distribution(generator);
        bool flip = rand < limit;

        bool assignment = (dist < 0) ^ flip;

        if(assignment){
            cout1->push_back(PointT(*it));
        }
        else{
            cout2->push_back(PointT(*it));
        }
        count++;
    }

    pcl::io::savePLYFile("out_test1.ply", *cout1, true);
    pcl::io::savePLYFile("out_test2.ply", *cout2, true);
}