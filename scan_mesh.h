#ifndef SCAN_MESH_H
#define SCAN_MESH_H

#include <string>
#include <vector>

namespace scanner {

struct PolarSample {
    float angleDegrees;
    float heightMm;
    float radiusMm;
};

struct Point3D {
    float x;
    float y;
    float z;
};

struct Triangle {
    Point3D a;
    Point3D b;
    Point3D c;
};

struct CropBounds {
    float minimumX;
    float maximumX;
    float minimumZ;
    float maximumZ;
    float minimumHeight;
    float maximumHeight;
};

bool contains(const CropBounds& bounds, float x, float height, float z);

bool validateTurntableCoverage(const std::vector<int>& anglesDegrees,
                               int maximumGapDegrees, std::string& error);

bool buildRadialMesh(const std::vector<PolarSample>& samples,
                     float maximumRadiusMm, float maximumHeightMm,
                     std::vector<Triangle>& triangles, std::string& error);

bool writeBinaryStl(const std::vector<Triangle>& triangles,
                    const std::string& filename, std::string& error);

} // namespace scanner

#endif
