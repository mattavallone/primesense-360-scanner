#include <OpenNI.h>
#include <opencv2/opencv.hpp>
#include <iostream>
#include <fstream>
#include <vector>
#include <cmath>
#include <cstring>

struct Point3D {
    float x, y, z;
};

struct Triangle {
    Point3D v1, v2, v3;
};

bool writeSTL(const std::vector<Triangle>& mesh, const std::string& filename) {
    std::ofstream outFile(filename);
    if (!outFile.is_open()) {
        std::cerr << "Error opening file: " << filename << std::endl;
        return false;
    }

    outFile << "solid OpenNI_Mesh" << std::endl;
    for (const auto& tri : mesh) {
        outFile << "  facet normal 0 0 0" << std::endl;
        outFile << "    outer loop" << std::endl;
        outFile << "      vertex " << tri.v1.x << " " << tri.v1.y << " " << tri.v1.z << std::endl;
        outFile << "      vertex " << tri.v2.x << " " << tri.v2.y << " " << tri.v2.z << std::endl;
        outFile << "      vertex " << tri.v3.x << " " << tri.v3.y << " " << tri.v3.z << std::endl;
        outFile << "    endloop" << std::endl;
        outFile << "  endfacet" << std::endl;
    }
    outFile << "endsolid OpenNI_Mesh" << std::endl;
    outFile.close();
    if (!outFile) {
        std::cerr << "Error writing file: " << filename << std::endl;
        return false;
    }
    return true;
}

std::vector<Point3D> depthToPointCloud(const openni::VideoFrameRef& depthFrame) {
    std::vector<Point3D> pointCloud;

    int width = depthFrame.getWidth();
    int height = depthFrame.getHeight();
    const openni::DepthPixel* depthData = (const openni::DepthPixel*)depthFrame.getData();

    float fx = 1.0f / 580.0f;  // Focal length (adjust based on camera model)
    float fy = 1.0f / 580.0f;
    float cx = width / 2.0f;
    float cy = height / 2.0f;

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            openni::DepthPixel depth = depthData[y * width + x];
            Point3D point;

            // If the depth is invalid, set the point to an indicator (e.g., z = 0)
            if (depth == 0) {
                point.x = point.y = point.z = 0; // You can use NaN or another marker if needed
            } else {
                point.z = (float)depth / 1000.0f;  // Convert depth to meters
                point.x = (x - cx) * point.z * fx;
                point.y = (y - cy) * point.z * fy;
            }

            pointCloud.push_back(point);
        }
    }

    return pointCloud;
}

std::vector<Triangle> generateMesh(const std::vector<Point3D>& pointCloud, int width, int height) {
    std::vector<Triangle> mesh;

    // Ensure that the pointCloud size matches the expected width * height
    if (pointCloud.size() != width * height) {
        std::cerr << "Error: Point cloud size does not match width * height" << std::endl;
        return mesh;
    }

    // Loop over the grid, avoiding the last row and column to prevent out-of-range accesses
    for (int y = 0; y < height - 1; ++y) {
        for (int x = 0; x < width - 1; ++x) {
            int idx1 = y * width + x;
            int idx2 = y * width + (x + 1);
            int idx3 = (y + 1) * width + x;
            int idx4 = (y + 1) * width + (x + 1);

            // Check if any points in the triangle have invalid depth (e.g., z == 0)
            if (pointCloud[idx1].z != 0 && pointCloud[idx2].z != 0 && pointCloud[idx3].z != 0) {
                mesh.push_back({pointCloud[idx1], pointCloud[idx2], pointCloud[idx3]});
            }
            if (pointCloud[idx3].z != 0 && pointCloud[idx2].z != 0 && pointCloud[idx4].z != 0) {
                mesh.push_back({pointCloud[idx3], pointCloud[idx2], pointCloud[idx4]});
            }
        }
    }

    return mesh;
}

void displayDepthMap(const openni::VideoFrameRef& depthFrame) {
    int width = depthFrame.getWidth();
    int height = depthFrame.getHeight();
    const openni::DepthPixel* depthData = (const openni::DepthPixel*)depthFrame.getData();

    cv::Mat depthMat(height, width, CV_16UC1, (void*)depthData);

    cv::Mat depthDisplay;
    depthMat.convertTo(depthDisplay, CV_8UC1, 255.0 / 10000);  // Scale depth to 0-255 range for display

    cv::applyColorMap(depthDisplay, depthDisplay, cv::COLORMAP_JET);

    cv::imshow("Depth Map", depthDisplay);
    cv::waitKey(0);  // Add a short delay to allow the window to refresh
}


int main(int argc, char* argv[]) {
    const bool headless = argc == 2 && std::strcmp(argv[1], "--headless") == 0;
    if (argc > 2 || (argc == 2 && !headless)) {
        std::cerr << "Usage: 3dscan_to_stl.exe [--headless]" << std::endl;
        return 2;
    }

    std::cout << "Initializing OpenNI..." << std::endl;
    openni::Status status = openni::OpenNI::initialize();
    if (status != openni::STATUS_OK) {
        std::cerr << "Failed to initialize OpenNI: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        return 1;
    }
    std::cout << "OpenNI initialized." << std::endl;

    openni::Device device;
    bool deviceOpened = false;
    bool streamCreated = false;
    bool streamStarted = false;
    openni::VideoStream depthStream;

    std::cout << "Opening depth camera..." << std::endl;
    status = device.open(openni::ANY_DEVICE);
    if (status != openni::STATUS_OK) {
        std::cerr << "Failed to open device: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        openni::OpenNI::shutdown();
        return 1;
    }
    deviceOpened = true;
    std::cout << "Depth camera opened." << std::endl;

    std::cout << "Creating depth stream..." << std::endl;
    status = depthStream.create(device, openni::SENSOR_DEPTH);
    if (status != openni::STATUS_OK) {
        std::cerr << "Failed to create depth stream: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        device.close();
        openni::OpenNI::shutdown();
        return 1;
    }
    streamCreated = true;

    std::cout << "Starting depth stream..." << std::endl;
    status = depthStream.start();
    if (status != openni::STATUS_OK) {
        std::cerr << "Failed to start depth stream: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        depthStream.destroy();
        device.close();
        openni::OpenNI::shutdown();
        return 1;
    }
    streamStarted = true;

    std::cout << "Waiting for a depth frame..." << std::endl;
    openni::VideoStream* streams[] = {&depthStream};
    int readyStreamIndex = 0;
    status = openni::OpenNI::waitForAnyStream(streams, 1, &readyStreamIndex,
                                               10000);
    if (status != openni::STATUS_OK) {
        std::cerr << "No depth frame became ready within 10 seconds (status "
                  << status << ")." << std::endl;
        depthStream.stop();
        depthStream.destroy();
        device.close();
        openni::OpenNI::shutdown();
        return 1;
    }

    openni::VideoFrameRef depthFrame;
    status = depthStream.readFrame(&depthFrame);
    if (status != openni::STATUS_OK || !depthFrame.isValid()) {
        std::cerr << "Failed to read depth frame: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        depthStream.stop();
        depthStream.destroy();
        device.close();
        openni::OpenNI::shutdown();
        return 1;
    }

    if (!headless) {
        displayDepthMap(depthFrame);
    }

    int width = depthFrame.getWidth();
    int height = depthFrame.getHeight();
    std::vector<Point3D> pointCloud = depthToPointCloud(depthFrame);

    std::vector<Triangle> mesh = generateMesh(pointCloud, width, height);

    const bool stlWritten = writeSTL(mesh, "output.stl");

    if (streamStarted) {
        depthStream.stop();
    }
    if (streamCreated) {
        depthStream.destroy();
    }
    if (deviceOpened) {
        device.close();
    }
    openni::OpenNI::shutdown();

    if (!stlWritten) {
        return 1;
    }
    std::cout << "STL file created: output.stl" << std::endl;
    return 0;
}
