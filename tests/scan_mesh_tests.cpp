#include "scan_mesh.h"

#include <cmath>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <string>
#include <vector>

namespace {

std::string vertexKey(const scanner::Point3D& point) {
    std::ostringstream key;
    key << static_cast<long long>(std::llround(point.x * 1000.0f)) << ","
        << static_cast<long long>(std::llround(point.y * 1000.0f)) << ","
        << static_cast<long long>(std::llround(point.z * 1000.0f));
    return key.str();
}

void addEdge(std::map<std::pair<std::string, std::string>, int>& edgeCounts,
             const scanner::Point3D& first,
             const scanner::Point3D& second) {
    std::string a = vertexKey(first);
    std::string b = vertexKey(second);
    if (b < a) {
        a.swap(b);
    }
    ++edgeCounts[std::make_pair(a, b)];
}

bool testClosedRadialMesh() {
    std::vector<scanner::PolarSample> samples;
    for (int angle = 0; angle < 360; ++angle) {
        const float radius = 52.0f +
            8.0f * std::cos(angle * 3.14159265358979323846f / 90.0f);
        for (int height = 6; height <= 150; height += 2) {
            scanner::PolarSample sample = {
                static_cast<float>(angle),
                static_cast<float>(height),
                radius
            };
            samples.push_back(sample);
        }
    }

    std::vector<scanner::Triangle> triangles;
    std::string error;
    if (!scanner::buildRadialMesh(samples, 180.0f, 300.0f,
                                  triangles, error)) {
        std::cerr << "Mesh generation failed: " << error << std::endl;
        return false;
    }
    if (triangles.empty()) {
        std::cerr << "Mesh generation returned no triangles." << std::endl;
        return false;
    }

    std::map<std::pair<std::string, std::string>, int> edgeCounts;
    double signedVolume = 0.0;
    for (std::size_t i = 0; i < triangles.size(); ++i) {
        const scanner::Triangle& triangle = triangles[i];
        addEdge(edgeCounts, triangle.a, triangle.b);
        addEdge(edgeCounts, triangle.b, triangle.c);
        addEdge(edgeCounts, triangle.c, triangle.a);
        const double crossX =
            triangle.b.y * triangle.c.z - triangle.b.z * triangle.c.y;
        const double crossY =
            triangle.b.z * triangle.c.x - triangle.b.x * triangle.c.z;
        const double crossZ =
            triangle.b.x * triangle.c.y - triangle.b.y * triangle.c.x;
        signedVolume += (triangle.a.x * crossX +
                         triangle.a.y * crossY +
                         triangle.a.z * crossZ) / 6.0;
    }
    for (std::map<std::pair<std::string, std::string>, int>::const_iterator
             edge = edgeCounts.begin();
         edge != edgeCounts.end(); ++edge) {
        if (edge->second != 2) {
            std::cerr << "Mesh is not closed; an edge occurs "
                      << edge->second << " times." << std::endl;
            return false;
        }
    }
    if (signedVolume <= 0.0) {
        std::cerr << "Mesh face winding does not enclose positive volume."
                  << std::endl;
        return false;
    }
    return true;
}

bool testRejectsInsufficientCoverage() {
    std::vector<scanner::PolarSample> samples;
    for (int angle = 0; angle < 80; ++angle) {
        for (int height = 6; height <= 100; height += 2) {
            scanner::PolarSample sample = {
                static_cast<float>(angle),
                static_cast<float>(height),
                40.0f
            };
            samples.push_back(sample);
        }
    }

    std::vector<scanner::Triangle> triangles;
    std::string error;
    if (scanner::buildRadialMesh(samples, 180.0f, 300.0f,
                                 triangles, error)) {
        std::cerr << "Mesh generation accepted insufficient angular coverage."
                  << std::endl;
        return false;
    }
    return !error.empty() && triangles.empty();
}

bool testBinaryStlOutput() {
    scanner::Triangle triangle = {
        {0.0f, 0.0f, 0.0f},
        {1.0f, 0.0f, 0.0f},
        {0.0f, 1.0f, 0.0f}
    };
    std::vector<scanner::Triangle> triangles(1, triangle);
    const std::string filename = "scan_mesh_test.stl";
    std::string error;
    if (!scanner::writeBinaryStl(triangles, filename, error)) {
        std::cerr << "STL write failed: " << error << std::endl;
        return false;
    }

    std::ifstream input(filename.c_str(), std::ios::binary);
    input.seekg(0, std::ios::end);
    const std::streamoff fileSize = input.tellg();
    input.close();
    std::remove(filename.c_str());
    if (fileSize != 134) {
        std::cerr << "Binary STL has unexpected size: " << fileSize
                  << std::endl;
        return false;
    }
    return true;
}

bool testCropBounds() {
    scanner::CropBounds bounds = {
        -35.0f, 35.0f, -40.0f, 40.0f, 10.0f, 600.0f
    };
    if (!scanner::contains(bounds, 0.0f, 450.0f, 0.0f) ||
        !scanner::contains(bounds, 0.0f, 60.0f, 0.0f) ||
        scanner::contains(bounds, 36.0f, 60.0f, 0.0f) ||
        scanner::contains(bounds, 0.0f, 60.0f, 41.0f) ||
        scanner::contains(bounds, 0.0f, 601.0f, 0.0f) ||
        scanner::contains(bounds, 0.0f, 60.0f,
                          std::numeric_limits<float>::quiet_NaN())) {
        std::cerr << "Crop volume accepted a point outside its bounds."
                  << std::endl;
        return false;
    }
    return true;
}

bool testTurntableCoverage() {
    std::vector<int> incomplete;
    for (int angle = 0; angle <= 90; angle += 15) {
        incomplete.push_back(angle);
    }
    std::string error;
    if (scanner::validateTurntableCoverage(incomplete, 30, error) ||
        error.empty()) {
        std::cerr << "Incomplete turntable coverage was accepted."
                  << std::endl;
        return false;
    }

    std::vector<int> fullTurn;
    for (int angle = 0; angle < 360; angle += 15) {
        fullTurn.push_back(angle);
    }
    if (!scanner::validateTurntableCoverage(fullTurn, 30, error)) {
        std::cerr << "A complete 15-degree scan was rejected: "
                  << error << std::endl;
        return false;
    }

    std::vector<int> duplicateAngles = fullTurn;
    duplicateAngles.push_back(0);
    if (!scanner::validateTurntableCoverage(duplicateAngles, 30, error)) {
        std::cerr << "Duplicate initial angle affected complete coverage."
                  << std::endl;
        return false;
    }
    return true;
}

} // namespace

int main() {
    if (!testClosedRadialMesh() || !testRejectsInsufficientCoverage() ||
        !testBinaryStlOutput() || !testCropBounds() ||
        !testTurntableCoverage()) {
        return 1;
    }
    std::cout << "Scan mesh tests passed." << std::endl;
    return 0;
}
