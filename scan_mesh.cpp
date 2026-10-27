#include "scan_mesh.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <fstream>
#include <limits>
#include <sstream>

namespace scanner {
namespace {

const int kAngleBins = 360;
const float kHeightStepMm = 2.0f;
const int kProfileSegments = 96;
const float kMinimumHeightMm = 5.0f;
const int kMaximumVerticalGapBins = 12;
const int kMaximumAngularGapBins = 24;
const float kPi = 3.14159265358979323846f;

struct Profile {
    bool valid;
    float bottomMm;
    float topMm;
    std::vector<float> radii;

    Profile() : valid(false), bottomMm(0.0f), topMm(0.0f),
                radii(kProfileSegments, 0.0f) {}
};

float median(std::vector<float>& values) {
    const std::size_t middle = values.size() / 2;
    std::nth_element(values.begin(), values.begin() + middle, values.end());
    float result = values[middle];
    if (values.size() % 2 == 0) {
        const float lower = *std::max_element(values.begin(),
                                              values.begin() + middle);
        result = (lower + result) * 0.5f;
    }
    return result;
}

bool sampleProfile(const std::vector<float>& column, float targetHeightMm,
                   float& radiusMm) {
    int lower = -1;
    int upper = -1;
    for (std::size_t i = 0; i < column.size(); ++i) {
        if (!std::isfinite(column[i])) {
            continue;
        }
        const float height = static_cast<float>(i) * kHeightStepMm;
        if (height <= targetHeightMm) {
            lower = static_cast<int>(i);
        }
        if (height >= targetHeightMm) {
            upper = static_cast<int>(i);
            break;
        }
    }

    if (lower < 0 || upper < 0 ||
        upper - lower > kMaximumVerticalGapBins) {
        return false;
    }
    if (lower == upper) {
        radiusMm = column[lower];
        return true;
    }

    const float lowerHeight = static_cast<float>(lower) * kHeightStepMm;
    const float fraction = (targetHeightMm - lowerHeight) /
        (static_cast<float>(upper - lower) * kHeightStepMm);
    radiusMm = column[lower] + (column[upper] - column[lower]) * fraction;
    return true;
}

bool buildProfile(const std::vector<float>& column, Profile& profile) {
    int first = -1;
    int last = -1;
    int validCount = 0;
    for (std::size_t i = 0; i < column.size(); ++i) {
        if (std::isfinite(column[i])) {
            if (first < 0) {
                first = static_cast<int>(i);
            }
            last = static_cast<int>(i);
            ++validCount;
        }
    }

    if (first < 0 || last - first < 10 || validCount < 8) {
        return false;
    }

    profile.bottomMm = static_cast<float>(first) * kHeightStepMm;
    profile.topMm = static_cast<float>(last) * kHeightStepMm;
    if (profile.topMm - profile.bottomMm < 20.0f) {
        return false;
    }

    int successfulSamples = 0;
    for (int i = 0; i < kProfileSegments; ++i) {
        const float fraction = static_cast<float>(i) /
            static_cast<float>(kProfileSegments - 1);
        const float height = profile.bottomMm +
            (profile.topMm - profile.bottomMm) * fraction;
        if (sampleProfile(column, height, profile.radii[i])) {
            ++successfulSamples;
        }
    }
    if (successfulSamples < kProfileSegments * 3 / 4) {
        return false;
    }

    for (int i = 0; i < kProfileSegments; ++i) {
        if (std::isfinite(profile.radii[i]) && profile.radii[i] > 0.0f) {
            continue;
        }
        int lower = i - 1;
        while (lower >= 0 &&
               (!std::isfinite(profile.radii[lower]) ||
                profile.radii[lower] <= 0.0f)) {
            --lower;
        }
        int upper = i + 1;
        while (upper < kProfileSegments &&
               (!std::isfinite(profile.radii[upper]) ||
                profile.radii[upper] <= 0.0f)) {
            ++upper;
        }
        if (lower < 0 || upper >= kProfileSegments ||
            upper - lower > 10) {
            return false;
        }
        const float fraction = static_cast<float>(i - lower) /
            static_cast<float>(upper - lower);
        profile.radii[i] = profile.radii[lower] +
            (profile.radii[upper] - profile.radii[lower]) * fraction;
    }

    profile.valid = true;
    return true;
}

Point3D polarPoint(float angleDegrees, float radiusMm, float heightMm) {
    const float angle = angleDegrees * kPi / 180.0f;
    Point3D point;
    point.x = radiusMm * std::sin(angle);
    point.y = heightMm;
    point.z = radiusMm * std::cos(angle);
    return point;
}

void appendTriangle(std::vector<Triangle>& triangles, const Point3D& a,
                    const Point3D& b, const Point3D& c) {
    Triangle triangle = {a, b, c};
    triangles.push_back(triangle);
}

void writeUint32(std::ofstream& output, std::uint32_t value) {
    const unsigned char bytes[4] = {
        static_cast<unsigned char>(value & 0xFF),
        static_cast<unsigned char>((value >> 8) & 0xFF),
        static_cast<unsigned char>((value >> 16) & 0xFF),
        static_cast<unsigned char>((value >> 24) & 0xFF)
    };
    output.write(reinterpret_cast<const char*>(bytes), sizeof(bytes));
}

void writeFloat(std::ofstream& output, float value) {
    std::uint32_t bits = 0;
    static_assert(sizeof(bits) == sizeof(value), "STL requires 32-bit floats");
    std::memcpy(&bits, &value, sizeof(bits));
    writeUint32(output, bits);
}

bool isFinite(const Point3D& point) {
    return std::isfinite(point.x) && std::isfinite(point.y) &&
           std::isfinite(point.z);
}

} // namespace

bool contains(const CropBounds& bounds, float x, float height, float z) {
    return std::isfinite(x) && std::isfinite(height) && std::isfinite(z) &&
           x >= bounds.minimumX && x <= bounds.maximumX &&
           z >= bounds.minimumZ && z <= bounds.maximumZ &&
           height >= bounds.minimumHeight &&
           height <= bounds.maximumHeight;
}

bool validateTurntableCoverage(const std::vector<int>& anglesDegrees,
                               int maximumGapDegrees, std::string& error) {
    error.clear();
    if (maximumGapDegrees <= 0 || maximumGapDegrees >= 360) {
        error = "The maximum turntable angle gap is invalid.";
        return false;
    }

    std::vector<int> angles;
    angles.reserve(anglesDegrees.size());
    for (std::size_t i = 0; i < anglesDegrees.size(); ++i) {
        int angle = anglesDegrees[i] % 360;
        if (angle < 0) {
            angle += 360;
        }
        angles.push_back(angle);
    }
    std::sort(angles.begin(), angles.end());
    angles.erase(std::unique(angles.begin(), angles.end()), angles.end());

    if (angles.size() < 12) {
        std::ostringstream message;
        message << "Only " << angles.size()
                << " unique angles are captured; capture at least 12 views "
                   "around a full turn before generating a preview.";
        error = message.str();
        return false;
    }

    int largestGap = 0;
    int gapStart = 0;
    for (std::size_t i = 0; i < angles.size(); ++i) {
        const int next = (i + 1 == angles.size())
            ? angles[0] + 360
            : angles[i + 1];
        const int gap = next - angles[i];
        if (gap > largestGap) {
            largestGap = gap;
            gapStart = angles[i];
        }
    }

    if (largestGap > maximumGapDegrees) {
        std::ostringstream message;
        message << "Turntable coverage has a " << largestGap
                << " degree gap after " << gapStart
                << " degrees. Capture a complete rotation with no gap over "
                << maximumGapDegrees << " degrees (24 views at 15-degree "
                   "steps recommended).";
        error = message.str();
        return false;
    }
    return true;
}

bool buildRadialMesh(const std::vector<PolarSample>& samples,
                     float maximumRadiusMm, float maximumHeightMm,
                     std::vector<Triangle>& triangles, std::string& error) {
    triangles.clear();
    error.clear();

    if (samples.empty()) {
        error = "No valid depth samples were collected.";
        return false;
    }
    if (!std::isfinite(maximumRadiusMm) ||
        !std::isfinite(maximumHeightMm) ||
        maximumRadiusMm <= 0.0f || maximumHeightMm <= kMinimumHeightMm) {
        error = "The scan volume dimensions are invalid.";
        return false;
    }

    const int heightBins = static_cast<int>(
        std::ceil(maximumHeightMm / kHeightStepMm)) + 1;
    std::vector<std::vector<float> > buckets(
        static_cast<std::size_t>(kAngleBins) * heightBins);

    for (std::size_t i = 0; i < samples.size(); ++i) {
        const PolarSample& sample = samples[i];
        if (!std::isfinite(sample.angleDegrees) ||
            !std::isfinite(sample.heightMm) ||
            !std::isfinite(sample.radiusMm) ||
            sample.heightMm < kMinimumHeightMm ||
            sample.heightMm > maximumHeightMm ||
            sample.radiusMm <= 1.0f ||
            sample.radiusMm > maximumRadiusMm) {
            continue;
        }

        float angle = std::fmod(sample.angleDegrees, 360.0f);
        if (angle < 0.0f) {
            angle += 360.0f;
        }
        const int angleBin = static_cast<int>(std::floor(angle + 0.5f)) %
            kAngleBins;
        const int heightBin = static_cast<int>(
            std::floor(sample.heightMm / kHeightStepMm + 0.5f));
        if (heightBin >= 0 && heightBin < heightBins) {
            buckets[static_cast<std::size_t>(angleBin) * heightBins +
                    heightBin].push_back(sample.radiusMm);
        }
    }

    std::vector<std::vector<float> > columns(
        kAngleBins, std::vector<float>(heightBins,
                                       std::numeric_limits<float>::quiet_NaN()));
    for (int angle = 0; angle < kAngleBins; ++angle) {
        for (int height = 0; height < heightBins; ++height) {
            std::vector<float>& values =
                buckets[static_cast<std::size_t>(angle) * heightBins + height];
            if (!values.empty()) {
                columns[angle][height] = median(values);
            }
        }
    }

    std::vector<Profile> profiles(kAngleBins);
    int validProfiles = 0;
    for (int angle = 0; angle < kAngleBins; ++angle) {
        if (buildProfile(columns[angle], profiles[angle])) {
            ++validProfiles;
        }
    }
    if (validProfiles < 180) {
        std::ostringstream message;
        message << "Only " << validProfiles
                << " of 360 turntable angles have usable data. Capture more "
                   "angles, check the turntable-axis calibration, or reduce "
                   "the object size.";
        error = message.str();
        return false;
    }

    const std::vector<Profile> measuredProfiles = profiles;
    for (int angle = 0; angle < kAngleBins; ++angle) {
        if (measuredProfiles[angle].valid) {
            continue;
        }

        int lower = 1;
        while (lower < kAngleBins &&
               !measuredProfiles[(angle - lower + kAngleBins) %
                                 kAngleBins].valid) {
            ++lower;
        }
        int upper = 1;
        while (upper < kAngleBins &&
               !measuredProfiles[(angle + upper) % kAngleBins].valid) {
            ++upper;
        }
        if (lower >= kAngleBins || upper >= kAngleBins ||
            lower + upper - 1 > kMaximumAngularGapBins) {
            error = "The scan has an angular gap larger than 24 degrees. "
                    "Capture more positions around the full turntable.";
            return false;
        }

        const Profile& left =
            measuredProfiles[(angle - lower + kAngleBins) % kAngleBins];
        const Profile& right = measuredProfiles[(angle + upper) % kAngleBins];
        const float fraction = static_cast<float>(lower) /
            static_cast<float>(lower + upper);
        Profile& profile = profiles[angle];
        profile.bottomMm = left.bottomMm +
            (right.bottomMm - left.bottomMm) * fraction;
        profile.topMm = left.topMm +
            (right.topMm - left.topMm) * fraction;
        for (int level = 0; level < kProfileSegments; ++level) {
            profile.radii[level] = left.radii[level] +
                (right.radii[level] - left.radii[level]) * fraction;
        }
        profile.valid = true;
    }

    std::vector<std::vector<Point3D> > vertices(
        kAngleBins, std::vector<Point3D>(kProfileSegments));
    float bottomCenterHeight = 0.0f;
    float topCenterHeight = 0.0f;
    for (int angle = 0; angle < kAngleBins; ++angle) {
        bottomCenterHeight += profiles[angle].bottomMm;
        topCenterHeight += profiles[angle].topMm;
        for (int level = 0; level < kProfileSegments; ++level) {
            const float fraction = static_cast<float>(level) /
                static_cast<float>(kProfileSegments - 1);
            const float height = profiles[angle].bottomMm +
                (profiles[angle].topMm - profiles[angle].bottomMm) * fraction;
            vertices[angle][level] =
                polarPoint(static_cast<float>(angle),
                           profiles[angle].radii[level], height);
        }
    }
    bottomCenterHeight /= static_cast<float>(kAngleBins);
    topCenterHeight /= static_cast<float>(kAngleBins);

    triangles.reserve(kAngleBins * (kProfileSegments - 1) * 2 +
                      kAngleBins * 2);
    for (int angle = 0; angle < kAngleBins; ++angle) {
        const int nextAngle = (angle + 1) % kAngleBins;
        for (int level = 0; level < kProfileSegments - 1; ++level) {
            const Point3D& a = vertices[angle][level];
            const Point3D& b = vertices[nextAngle][level];
            const Point3D& c = vertices[nextAngle][level + 1];
            const Point3D& d = vertices[angle][level + 1];
            appendTriangle(triangles, a, b, c);
            appendTriangle(triangles, a, c, d);
        }

        const Point3D bottomCenter = {0.0f, bottomCenterHeight, 0.0f};
        const Point3D topCenter = {0.0f, topCenterHeight, 0.0f};
        appendTriangle(triangles, vertices[angle][0], bottomCenter,
                       vertices[nextAngle][0]);
        appendTriangle(triangles, vertices[angle][kProfileSegments - 1],
                       vertices[nextAngle][kProfileSegments - 1], topCenter);
    }

    return true;
}

bool writeBinaryStl(const std::vector<Triangle>& triangles,
                    const std::string& filename, std::string& error) {
    error.clear();
    if (triangles.empty() ||
        triangles.size() > std::numeric_limits<std::uint32_t>::max()) {
        error = "The mesh is empty or too large to write as STL.";
        return false;
    }

    std::ofstream output(filename.c_str(), std::ios::binary);
    if (!output) {
        error = "Could not open the selected STL file for writing.";
        return false;
    }

    char header[80] = {};
    const char description[] = "PrimeSense turntable scan; units: millimeters";
    std::copy(description, description + sizeof(description) - 1, header);
    output.write(header, sizeof(header));
    writeUint32(output, static_cast<std::uint32_t>(triangles.size()));

    for (std::size_t i = 0; i < triangles.size(); ++i) {
        const Triangle& triangle = triangles[i];
        if (!isFinite(triangle.a) || !isFinite(triangle.b) ||
            !isFinite(triangle.c)) {
            error = "The mesh contains a non-finite vertex.";
            return false;
        }

        const float ux = triangle.b.x - triangle.a.x;
        const float uy = triangle.b.y - triangle.a.y;
        const float uz = triangle.b.z - triangle.a.z;
        const float vx = triangle.c.x - triangle.a.x;
        const float vy = triangle.c.y - triangle.a.y;
        const float vz = triangle.c.z - triangle.a.z;
        float nx = uy * vz - uz * vy;
        float ny = uz * vx - ux * vz;
        float nz = ux * vy - uy * vx;
        const float length = std::sqrt(nx * nx + ny * ny + nz * nz);
        if (length > 0.0f) {
            nx /= length;
            ny /= length;
            nz /= length;
        }

        writeFloat(output, nx);
        writeFloat(output, ny);
        writeFloat(output, nz);
        writeFloat(output, triangle.a.x);
        writeFloat(output, triangle.a.y);
        writeFloat(output, triangle.a.z);
        writeFloat(output, triangle.b.x);
        writeFloat(output, triangle.b.y);
        writeFloat(output, triangle.b.z);
        writeFloat(output, triangle.c.x);
        writeFloat(output, triangle.c.y);
        writeFloat(output, triangle.c.z);
        const unsigned char attributeBytes[2] = {0, 0};
        output.write(reinterpret_cast<const char*>(attributeBytes),
                     sizeof(attributeBytes));
    }

    if (!output) {
        error = "An error occurred while writing the STL file.";
        return false;
    }
    return true;
}

} // namespace scanner
