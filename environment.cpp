/* \author Aaron Brown, Bhagyath*/
// Create simple 3d highway environment using PCL
// for exploring self-driving car sensors

#include "render/render.h"
#include "processPointClouds.h"
#include "processPointClouds.cpp"   // template implementation

#include <iostream>
#include <vector>
#include <string>
#include <filesystem>


// CityBlock: process ONE frame

void cityBlock(pcl::visualization::PCLVisualizer::Ptr& viewer,
               ProcessPointClouds<pcl::PointXYZI>* pointProcessorI,
               const pcl::PointCloud<pcl::PointXYZI>::Ptr& inputCloud)
{
    // 1) Filter 
    float filterRes = 0.2f;
    Eigen::Vector4f minPoint(-10.0f, -6.0f, -3.0f, 1.0f);
    Eigen::Vector4f maxPoint( 30.0f,  6.0f,  3.0f, 1.0f);

    pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud =
        pointProcessorI->FilterCloud(inputCloud, filterRes, minPoint, maxPoint);



    // 2) Segment: road plane vs obstacles
    auto segmentCloud = pointProcessorI->SegmentPlane(filteredCloud, 100, 0.2);

    renderPointCloud(viewer, segmentCloud.second, "planeCloud", Color(0, 1, 0)); // road
    renderPointCloud(viewer, segmentCloud.first,  "obstCloud",  Color(1, 0, 0)); // obstacles

    // 3) Cluster the obstacle cloud
    std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> cloudClusters =
        pointProcessorI->Clustering(segmentCloud.first, 0.5, 15, 500);

    int clusterId = 0;
    std::vector<Color> colors = { Color(1,0,0), Color(1,1,0), Color(0,0,1) };

    for (auto cluster : cloudClusters)
    {
        renderPointCloud(viewer,
                         cluster,
                         "cluster" + std::to_string(clusterId),
                         colors[clusterId % colors.size()]);

        Box box = pointProcessorI->BoundingBox(cluster);
        renderBox(viewer, box, clusterId);

        ++clusterId;
    }
}

// setAngle: SWITCH CAMERA ANGLE {XY, TopDown, Side, FPS}
void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{
    viewer->setBackgroundColor(0, 0, 0);
    viewer->initCameraParameters();

    int distance = 16;
    switch (setAngle)
    {
        case XY:      viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown: viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side:    viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS:     viewer->setCameraPosition(-10, 0, 0, 0, 0, 1); break;
    }

    if (setAngle != FPS)
        viewer->addCoordinateSystem(1.0);
}

int main(int argc, char** argv)
{
    std::cout << "starting environment" << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    CameraAngle setAngle = FPS;
    initCamera(setAngle, viewer);

    // Create the point processor ONCE
    ProcessPointClouds<pcl::PointXYZI>* pointProcessorI = new ProcessPointClouds<pcl::PointXYZI>();

    // Stream pcd files
    std::string dataPath = "../src/sensors/data/pcd/data_1";
    std::vector<std::filesystem::path> stream = pointProcessorI->streamPcd(dataPath);
    auto streamIterator = stream.begin();

    pcl::PointCloud<pcl::PointXYZI>::Ptr inputCloudI;

    while (!viewer->wasStopped())
    {
        // Clear viewer each frame (this is why you were seeing “old one only” before)
        viewer->removeAllPointClouds();
        viewer->removeAllShapes();

        // Load next frame
        inputCloudI = pointProcessorI->loadPcd(streamIterator->string());

        // Run pipeline on this frame
        cityBlock(viewer, pointProcessorI, inputCloudI);

        // Advance iterator, loop back to beginning
        streamIterator++;
        if (streamIterator == stream.end())
            streamIterator = stream.begin();

        viewer->spinOnce(100);
    }

    delete pointProcessorI;
    return 0;
}
