#define NOMINMAX
#include <Windows.h>
#include <CommDlg.h>
#include <OpenNI.h>
#include <opencv2/opencv.hpp>

#include "scan_mesh.h"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <limits>
#include <sstream>
#include <string>
#include <vector>

namespace {

const float kMaximumRadiusMm = 180.0f;
const float kMaximumHeightMm = 600.0f;
const int kMinimumCameraDepthMm = 500;
const int kMaximumCameraDepthMm = 10000;
const int kTurntableStepDegrees = 15;
const float kPi = 3.14159265358979323846f;

struct CapturedDepthFrame {
    std::vector<openni::DepthPixel> depth;
    int width;
    int height;
    int angleDegrees;
};

enum UiAction {
    ACTION_NONE,
    ACTION_SET_AXIS,
    ACTION_CAPTURE,
    ACTION_ANGLE_BACK,
    ACTION_ANGLE_FORWARD,
    ACTION_UNDO,
    ACTION_CLEAR,
    ACTION_GENERATE,
    ACTION_SAVE,
    ACTION_RESET_RANGES,
    ACTION_POSITION_CROP
};

struct ScanControls {
    int action;
    bool waitingForAxis;
    bool waitingForCropCenter;
    bool hasAxis;
    bool cropEnabled;
    float axisX;
    float axisY;
    float axisZ;
    float cropCenterX;
    float cropCenterZ;
    int axisPixelX;
    int axisPixelY;
    int currentAngleDegrees;
    int depthWidth;
    int depthHeight;
    std::vector<openni::DepthPixel> latestDepth;
    std::vector<CapturedDepthFrame> captures;
    std::vector<scanner::Triangle> mesh;
    scanner::CropBounds cropBounds;
    int minimumCameraDepthMm;
    int maximumCameraDepthMm;
    cv::Mat cachedPreview;
    bool previewDirty;
    int cachedPreviewWidth;
    int cachedPreviewHeight;
    std::string status;
    float previewYaw;
    float previewPitch;
    bool draggingPreview;
    int previousMouseX;
    int previousMouseY;
    int canvasWidth;
    int canvasHeight;
    cv::Rect depthImageRect;
    cv::Rect previewImageRect;
    cv::Rect cropToggleRect;
    cv::Rect cropSliderTracks[4];
    cv::Rect buttons[10];
    int draggingCropHandle;
    openni::VideoStream* stream;

    ScanControls()
        : action(ACTION_NONE), waitingForAxis(false),
          waitingForCropCenter(false), hasAxis(false),
          cropEnabled(true),
          axisX(0.0f), axisY(0.0f), axisZ(0.0f),
          cropCenterX(0.0f), cropCenterZ(0.0f),
          axisPixelX(0), axisPixelY(0),
          currentAngleDegrees(0), depthWidth(0), depthHeight(0),
          cropBounds{-180.0f, 180.0f, -180.0f, 180.0f, 0.0f, 600.0f},
          minimumCameraDepthMm(500), maximumCameraDepthMm(3500),
          previewDirty(true), cachedPreviewWidth(0), cachedPreviewHeight(0),
          draggingCropHandle(-1),
          previewYaw(-0.45f), previewPitch(-0.25f),
          draggingPreview(false), previousMouseX(0), previousMouseY(0),
          canvasWidth(1200), canvasHeight(700), stream(nullptr) {}
};

bool selectDepthMode(openni::VideoStream& stream) {
    const openni::Array<openni::VideoMode>& modes =
        stream.getSensorInfo().getSupportedVideoModes();
    int selected = -1;
    int selectedArea = 0;
    int selectedFps = 0;

    for (int i = 0; i < modes.getSize(); ++i) {
        const openni::VideoMode& mode = modes[i];
        if (mode.getPixelFormat() != openni::PIXEL_FORMAT_DEPTH_1_MM) {
            continue;
        }
        const int width = mode.getResolutionX();
        const int height = mode.getResolutionY();
        const int area = width * height;
        if (width <= 0 || height <= 0 ||
            width > 640 || height > 480) {
            continue;
        }
        if (area > selectedArea ||
            (area == selectedArea && mode.getFps() > selectedFps)) {
            selected = i;
            selectedArea = area;
            selectedFps = mode.getFps();
        }
    }

    if (selected < 0) {
        std::cerr << "The depth sensor has no supported DEPTH_1_MM mode "
                     "at or below 640x480." << std::endl;
        return false;
    }
    if (stream.setVideoMode(modes[selected]) != openni::STATUS_OK) {
        std::cerr << "Could not select the depth video mode: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        return false;
    }
    return true;
}

cv::Mat makeDepthDisplay(const std::vector<openni::DepthPixel>& depth,
                         int width, int height, int minimumDepth,
                         int maximumDepth) {
    if (width <= 0 || height <= 0 ||
        depth.size() != static_cast<std::size_t>(width) * height) {
        return cv::Mat();
    }

    cv::Mat source(height, width, CV_16UC1,
                   const_cast<openni::DepthPixel*>(depth.data()));
    cv::Mat clipped;
    cv::max(source, minimumDepth, clipped);
    cv::min(clipped, maximumDepth, clipped);

    cv::Mat grayscale;
    const double depthRange = maximumDepth - minimumDepth;
    clipped.convertTo(grayscale, CV_8UC1, 255.0 / depthRange,
                      -minimumDepth * 255.0 / depthRange);
    cv::Mat colorized;
    cv::applyColorMap(grayscale, colorized, cv::COLORMAP_TURBO);
    cv::Mat invalidMask =
        (source < minimumDepth) | (source > maximumDepth);
    colorized.setTo(cv::Scalar(24, 27, 33), invalidMask);
    return colorized;
}

scanner::Point3D rotatePreviewPoint(const scanner::Point3D& point,
                                   float yaw, float pitch) {
    const float cosYaw = std::cos(yaw);
    const float sinYaw = std::sin(yaw);
    const float x = point.x * cosYaw + point.z * sinYaw;
    const float z = -point.x * sinYaw + point.z * cosYaw;
    const float cosPitch = std::cos(pitch);
    const float sinPitch = std::sin(pitch);
    scanner::Point3D rotated = {
        x,
        point.y * cosPitch - z * sinPitch,
        point.y * sinPitch + z * cosPitch
    };
    return rotated;
}

cv::Mat renderMeshPreview(const std::vector<scanner::Triangle>& mesh,
                          int width, int height, float yaw, float pitch) {
    cv::Mat image(height, width, CV_8UC3, cv::Scalar(24, 27, 33));
    if (mesh.empty()) {
        const std::string message =
            "Capture a complete turn, then generate the preview";
        cv::putText(image, message, cv::Point(18, height / 2 - 8),
                    cv::FONT_HERSHEY_SIMPLEX, 0.56,
                    cv::Scalar(188, 196, 207), 1, cv::LINE_AA);
        cv::putText(image, "The preview outlines the reconstructed object.",
                    cv::Point(18, height / 2 + 22),
                    cv::FONT_HERSHEY_SIMPLEX, 0.52,
                    cv::Scalar(140, 150, 164), 1, cv::LINE_AA);
        return image;
    }

    float minX = std::numeric_limits<float>::max();
    float maxX = -std::numeric_limits<float>::max();
    float minY = std::numeric_limits<float>::max();
    float maxY = -std::numeric_limits<float>::max();
    float minZ = std::numeric_limits<float>::max();
    float maxZ = -std::numeric_limits<float>::max();

    std::vector<scanner::Triangle> rotated;
    rotated.reserve(mesh.size());
    for (std::size_t i = 0; i < mesh.size(); ++i) {
        scanner::Triangle triangle = {
            rotatePreviewPoint(mesh[i].a, yaw, pitch),
            rotatePreviewPoint(mesh[i].b, yaw, pitch),
            rotatePreviewPoint(mesh[i].c, yaw, pitch)
        };
        const scanner::Point3D points[] = {
            triangle.a, triangle.b, triangle.c
        };
        for (int p = 0; p < 3; ++p) {
            minX = std::min(minX, points[p].x);
            maxX = std::max(maxX, points[p].x);
            minY = std::min(minY, points[p].y);
            maxY = std::max(maxY, points[p].y);
            minZ = std::min(minZ, points[p].z);
            maxZ = std::max(maxZ, points[p].z);
        }
        rotated.push_back(triangle);
    }

    const float centerX = (minX + maxX) * 0.5f;
    const float centerY = (minY + maxY) * 0.5f;
    const float centerZ = (minZ + maxZ) * 0.5f;
    const float extent = std::max(maxX - minX, maxY - minY);
    const float scale = 0.82f * std::min(width, height) /
        std::max(1.0f, extent);
    const cv::Vec3f light(0.25f, 0.55f, 0.79f);
    cv::Mat depthBuffer(height, width, CV_32FC1,
                        cv::Scalar(-std::numeric_limits<float>::max()));

    for (std::size_t i = 0; i < rotated.size(); ++i) {
        const scanner::Triangle& triangle = rotated[i];
        const scanner::Point3D points[] = {
            triangle.a, triangle.b, triangle.c
        };
        cv::Point2f screen[3];
        for (int p = 0; p < 3; ++p) {
            screen[p] = cv::Point2f(
                width * 0.5f + (points[p].x - centerX) * scale,
                height * 0.5f - (points[p].y - centerY) * scale);
        }

        const cv::Vec3f edge1(
            triangle.b.x - triangle.a.x,
            triangle.b.y - triangle.a.y,
            triangle.b.z - triangle.a.z);
        const cv::Vec3f edge2(
            triangle.c.x - triangle.a.x,
            triangle.c.y - triangle.a.y,
            triangle.c.z - triangle.a.z);
        cv::Vec3f normal = edge1.cross(edge2);
        const float normalLength = static_cast<float>(cv::norm(normal));
        const float diffuse = normalLength > 0.0f
            ? std::max(0.0f, static_cast<float>(normal.dot(light)) /
                               normalLength)
            : 0.0f;
        const unsigned char shade = static_cast<unsigned char>(
            55.0f + 150.0f * diffuse);
        const cv::Vec3b color(
            static_cast<unsigned char>(shade * 0.78),
            static_cast<unsigned char>(shade * 0.9), shade);

        const float denominator =
            (screen[1].y - screen[2].y) *
                (screen[0].x - screen[2].x) +
            (screen[2].x - screen[1].x) *
                (screen[0].y - screen[2].y);
        if (std::abs(denominator) < 0.0001f) {
            continue;
        }

        const int left = std::max(0, static_cast<int>(std::floor(
            std::min(screen[0].x, std::min(screen[1].x, screen[2].x)))));
        const int right = std::min(width - 1, static_cast<int>(std::ceil(
            std::max(screen[0].x, std::max(screen[1].x, screen[2].x)))));
        const int top = std::max(0, static_cast<int>(std::floor(
            std::min(screen[0].y, std::min(screen[1].y, screen[2].y)))));
        const int bottom = std::min(height - 1, static_cast<int>(std::ceil(
            std::max(screen[0].y, std::max(screen[1].y, screen[2].y)))));

        for (int py = top; py <= bottom; ++py) {
            cv::Vec3b* imageRow = image.ptr<cv::Vec3b>(py);
            float* depthRow = depthBuffer.ptr<float>(py);
            for (int px = left; px <= right; ++px) {
                const float sampleX = px + 0.5f;
                const float sampleY = py + 0.5f;
                const float weightA =
                    ((screen[1].y - screen[2].y) *
                         (sampleX - screen[2].x) +
                     (screen[2].x - screen[1].x) *
                         (sampleY - screen[2].y)) / denominator;
                const float weightB =
                    ((screen[2].y - screen[0].y) *
                         (sampleX - screen[2].x) +
                     (screen[0].x - screen[2].x) *
                         (sampleY - screen[2].y)) / denominator;
                const float weightC = 1.0f - weightA - weightB;
                if (weightA < -0.001f || weightB < -0.001f ||
                    weightC < -0.001f) {
                    continue;
                }

                const float depth = weightA * points[0].z +
                    weightB * points[1].z + weightC * points[2].z;
                if (depth <= depthRow[px]) {
                    continue;
                }
                depthRow[px] = depth;
                imageRow[px] = color;
            }
        }
    }

    cv::Mat silhouette(height, width, CV_8UC1, cv::Scalar(0));
    for (int y = 0; y < height; ++y) {
        const float* depthRow = depthBuffer.ptr<float>(y);
        unsigned char* maskRow = silhouette.ptr<unsigned char>(y);
        for (int x = 0; x < width; ++x) {
            if (depthRow[x] > -std::numeric_limits<float>::max()) {
                maskRow[x] = 255;
            }
        }
    }
    std::vector<std::vector<cv::Point> > contours;
    cv::findContours(silhouette, contours, cv::RETR_EXTERNAL,
                     cv::CHAIN_APPROX_SIMPLE);
    cv::drawContours(image, contours, -1, cv::Scalar(235, 244, 250),
                     1, cv::LINE_AA);

    std::ostringstream dimensions;
    dimensions << static_cast<int>(std::ceil(maxX - minX)) << " x "
               << static_cast<int>(std::ceil(maxY - minY)) << " x "
               << static_cast<int>(std::ceil(maxZ - minZ)) << " mm";
    cv::putText(image, dimensions.str(), cv::Point(12, height - 12),
                cv::FONT_HERSHEY_SIMPLEX, 0.45,
                cv::Scalar(196, 202, 212), 1, cv::LINE_AA);
    return image;
}

void drawButton(cv::Mat& canvas, const cv::Rect& rectangle,
                const std::string& label, bool active = false) {
    const cv::Scalar fill = active
        ? cv::Scalar(55, 121, 82)
        : cv::Scalar(57, 64, 74);
    cv::rectangle(canvas, rectangle, fill, cv::FILLED);
    cv::rectangle(canvas, rectangle, cv::Scalar(91, 100, 113), 1);
    int baseline = 0;
    const cv::Size textSize = cv::getTextSize(
        label, cv::FONT_HERSHEY_SIMPLEX, 0.42, 1, &baseline);
    const cv::Point origin(
        rectangle.x + (rectangle.width - textSize.width) / 2,
        rectangle.y + (rectangle.height + textSize.height) / 2 - 1);
    cv::putText(canvas, label, origin, cv::FONT_HERSHEY_SIMPLEX, 0.42,
                cv::Scalar(245, 247, 250), 1, cv::LINE_AA);
}

void drawWrappedStatus(cv::Mat& canvas, const std::string& text,
                       int x, int y, int maximumWidth) {
    std::istringstream words(text);
    std::string word;
    std::string line;
    int lineNumber = 0;
    while (words >> word) {
        const std::string candidate = line.empty()
            ? word
            : line + " " + word;
        const int textWidth = cv::getTextSize(
            candidate, cv::FONT_HERSHEY_SIMPLEX, 0.43, 1, nullptr).width;
        if (!line.empty() && textWidth > maximumWidth) {
            cv::putText(canvas, line, cv::Point(x, y + lineNumber * 18),
                        cv::FONT_HERSHEY_SIMPLEX, 0.43,
                        cv::Scalar(162, 195, 176), 1, cv::LINE_AA);
            ++lineNumber;
            line = word;
            if (lineNumber == 2) {
                line += "...";
                break;
            }
        } else {
            line = candidate;
        }
    }
    if (!line.empty() && lineNumber < 2) {
        cv::putText(canvas, line, cv::Point(x, y + lineNumber * 18),
                    cv::FONT_HERSHEY_SIMPLEX, 0.43,
                    cv::Scalar(162, 195, 176), 1, cv::LINE_AA);
    }
}

float cropRangeMinimum(const ScanControls& controls, int rangeIndex) {
    switch (rangeIndex) {
    case 0:
        return controls.cropBounds.minimumX;
    case 1:
        return controls.cropBounds.minimumZ;
    case 2:
        return controls.cropBounds.minimumHeight;
    default:
        return static_cast<float>(controls.minimumCameraDepthMm);
    }
}

float cropRangeMaximum(const ScanControls& controls, int rangeIndex) {
    switch (rangeIndex) {
    case 0:
        return controls.cropBounds.maximumX;
    case 1:
        return controls.cropBounds.maximumZ;
    case 2:
        return controls.cropBounds.maximumHeight;
    default:
        return static_cast<float>(controls.maximumCameraDepthMm);
    }
}

int cropRangeGlobalMinimum(int rangeIndex) {
    return rangeIndex == 2 ? 0
        : rangeIndex == 3 ? kMinimumCameraDepthMm : -180;
}

int cropRangeGlobalMaximum(int rangeIndex) {
    return rangeIndex == 2 ? static_cast<int>(kMaximumHeightMm)
        : rangeIndex == 3 ? kMaximumCameraDepthMm : 180;
}

int cropRangeValueToX(const cv::Rect& track, float value,
                      int rangeMinimum, int rangeMaximum) {
    const float fraction = (value - rangeMinimum) /
        static_cast<float>(rangeMaximum - rangeMinimum);
    return track.x + cvRound(fraction * (track.width - 1));
}

float cropRangeValueAtX(const cv::Rect& track, int x,
                        int rangeMinimum, int rangeMaximum) {
    const int clampedX = std::max(track.x,
        std::min(track.x + track.width - 1, x));
    const float fraction = static_cast<float>(clampedX - track.x) /
        static_cast<float>(std::max(1, track.width - 1));
    return rangeMinimum + fraction * (rangeMaximum - rangeMinimum);
}

void setCropRangeValue(ScanControls& controls, int rangeIndex,
                       bool setMinimum, int value) {
    const int lowerBound = cropRangeGlobalMinimum(rangeIndex);
    const int upperBound = cropRangeGlobalMaximum(rangeIndex);
    value = std::max(lowerBound, std::min(upperBound, value));
    const int minimumGap = rangeIndex == 3 ? 100 : 5;

    if (rangeIndex == 0) {
        if (setMinimum) {
            controls.cropBounds.minimumX = static_cast<float>(
                std::min(value,
                    static_cast<int>(controls.cropBounds.maximumX) -
                        minimumGap));
        } else {
            controls.cropBounds.maximumX = static_cast<float>(
                std::max(value,
                    static_cast<int>(controls.cropBounds.minimumX) +
                        minimumGap));
        }
    } else if (rangeIndex == 1) {
        if (setMinimum) {
            controls.cropBounds.minimumZ = static_cast<float>(
                std::min(value,
                    static_cast<int>(controls.cropBounds.maximumZ) -
                        minimumGap));
        } else {
            controls.cropBounds.maximumZ = static_cast<float>(
                std::max(value,
                    static_cast<int>(controls.cropBounds.minimumZ) +
                        minimumGap));
        }
    } else if (rangeIndex == 2) {
        if (setMinimum) {
            controls.cropBounds.minimumHeight = static_cast<float>(
                std::min(value,
                    static_cast<int>(controls.cropBounds.maximumHeight) -
                        minimumGap));
        } else {
            controls.cropBounds.maximumHeight = static_cast<float>(
                std::max(value,
                    static_cast<int>(controls.cropBounds.minimumHeight) +
                        minimumGap));
        }
    } else if (setMinimum) {
        controls.minimumCameraDepthMm = std::min(
            value, controls.maximumCameraDepthMm - minimumGap);
    } else {
        controls.maximumCameraDepthMm = std::max(
            value, controls.minimumCameraDepthMm + minimumGap);
    }
}

void drawCropRange(cv::Mat& canvas, ScanControls& controls, int rangeIndex,
                   int y, const char* label) {
    const int trackLeft = 226;
    const int trackRight = canvas.cols - 24;
    controls.cropSliderTracks[rangeIndex] = cv::Rect(
        trackLeft, y + 4, std::max(1, trackRight - trackLeft + 1), 16);
    const cv::Rect& track = controls.cropSliderTracks[rangeIndex];
    const int globalMinimum = cropRangeGlobalMinimum(rangeIndex);
    const int globalMaximum = cropRangeGlobalMaximum(rangeIndex);
    const int minimumX = cropRangeValueToX(
        track, cropRangeMinimum(controls, rangeIndex),
        globalMinimum, globalMaximum);
    const int maximumX = cropRangeValueToX(
        track, cropRangeMaximum(controls, rangeIndex),
        globalMinimum, globalMaximum);
    const int centerY = track.y + track.height / 2;

    cv::putText(canvas, label, cv::Point(16, y + 16),
                cv::FONT_HERSHEY_SIMPLEX, 0.39,
                cv::Scalar(203, 210, 220), 1, cv::LINE_AA);
    std::ostringstream values;
    values << static_cast<int>(std::lround(
                  cropRangeMinimum(controls, rangeIndex)))
           << " .. "
           << static_cast<int>(std::lround(
                  cropRangeMaximum(controls, rangeIndex)))
           << " mm";
    cv::putText(canvas, values.str(), cv::Point(115, y + 16),
                cv::FONT_HERSHEY_SIMPLEX, 0.37,
                cv::Scalar(163, 174, 188), 1, cv::LINE_AA);

    cv::line(canvas, cv::Point(track.x, centerY),
             cv::Point(track.x + track.width - 1, centerY),
             cv::Scalar(76, 82, 92), 3, cv::LINE_AA);
    cv::line(canvas, cv::Point(minimumX, centerY),
             cv::Point(maximumX, centerY),
             cv::Scalar(63, 154, 111), 4, cv::LINE_AA);
    cv::circle(canvas, cv::Point(minimumX, centerY), 6,
               cv::Scalar(231, 238, 245), cv::FILLED, cv::LINE_AA);
    cv::circle(canvas, cv::Point(maximumX, centerY), 6,
               cv::Scalar(231, 238, 245), cv::FILLED, cv::LINE_AA);
}

void drawCropVolumeOverlay(cv::Mat& canvas, const ScanControls& controls);

cv::Mat makeCanvas(const cv::Mat& depthDisplay, ScanControls& controls) {
    controls.canvasWidth = std::min(
        1240, std::max(800, GetSystemMetrics(SM_CXSCREEN) - 80));
    controls.canvasHeight = std::min(
        820, std::max(540, GetSystemMetrics(SM_CYSCREEN) - 130));

    const bool compactControls = true;
    const int panelWidth = (controls.canvasWidth - 48) / 2;
    const int panelTop = 76;
    const int viewHeight = controls.canvasHeight -
        (compactControls ? 444 : 400);
    const cv::Rect depthPanel(16, 68, panelWidth, viewHeight + 22);
    const cv::Rect previewPanel(32 + panelWidth, 68, panelWidth,
                                viewHeight + 22);
    const cv::Rect depthViewArea(depthPanel.x + 10, panelTop,
                                 panelWidth - 20, viewHeight);
    const cv::Rect previewViewArea(previewPanel.x + 10, panelTop,
                                   panelWidth - 20, viewHeight);

    const double depthScale = depthDisplay.empty() ? 1.0 : std::min(
        static_cast<double>(depthViewArea.width) / depthDisplay.cols,
        static_cast<double>(depthViewArea.height) / depthDisplay.rows);
    const int depthWidth = depthDisplay.empty()
        ? 1 : cvRound(depthDisplay.cols * depthScale);
    const int depthHeight = depthDisplay.empty()
        ? 1 : cvRound(depthDisplay.rows * depthScale);
    controls.depthImageRect = cv::Rect(
        depthViewArea.x + (depthViewArea.width - depthWidth) / 2,
        depthViewArea.y + (depthViewArea.height - depthHeight) / 2,
        depthWidth, depthHeight);

    if (controls.previewDirty ||
        controls.cachedPreviewWidth != previewViewArea.width ||
        controls.cachedPreviewHeight != previewViewArea.height) {
        controls.cachedPreview = renderMeshPreview(
            controls.mesh, previewViewArea.width, previewViewArea.height,
            controls.previewYaw, controls.previewPitch);
        controls.cachedPreviewWidth = previewViewArea.width;
        controls.cachedPreviewHeight = previewViewArea.height;
        controls.previewDirty = false;
    }
    controls.previewImageRect = previewViewArea;

    const int rowY = controls.canvasHeight -
        (compactControls ? 326 : 290);
    const int buttonHeight = 36;
    const int gap = 8;
    const int buttonY = rowY + 4 * 22 + 10;
    if (compactControls) {
        const int buttonWidth = (controls.canvasWidth - 32 - 4 * gap) / 5;
        for (int i = 0; i < 10; ++i) {
            const int column = i % 5;
            const int row = i / 5;
            controls.buttons[i] = cv::Rect(
                16 + column * (buttonWidth + gap),
                buttonY + row * (buttonHeight + gap),
                buttonWidth, buttonHeight);
        }
    } else {
        int x = 16;
        const int widths[] = {155, 177, 82, 82, 72, 82, 185, 104, 112, 132};
        for (int i = 0; i < 10; ++i) {
            controls.buttons[i] = cv::Rect(x, buttonY, widths[i], buttonHeight);
            x += widths[i] + gap;
        }
    }

    cv::Mat canvas(controls.canvasHeight, controls.canvasWidth, CV_8UC3,
                   cv::Scalar(31, 35, 41));
    cv::rectangle(canvas, cv::Rect(0, 0, canvas.cols, 58),
                  cv::Scalar(38, 43, 51), cv::FILLED);
    cv::putText(canvas, "PrimeSense 360 Scan",
                cv::Point(18, 27), cv::FONT_HERSHEY_SIMPLEX, 0.7,
                cv::Scalar(244, 246, 249), 1, cv::LINE_AA);
    cv::putText(canvas, "Manual turntable capture",
                cv::Point(19, 48), cv::FONT_HERSHEY_SIMPLEX, 0.43,
                cv::Scalar(157, 167, 180), 1, cv::LINE_AA);
    cv::putText(canvas, "Position crop, then click target center in depth",
                cv::Point(16, rowY - 5), cv::FONT_HERSHEY_SIMPLEX, 0.37,
                cv::Scalar(144, 154, 168), 1, cv::LINE_AA);
    controls.cropToggleRect = cv::Rect(
        controls.canvasWidth - 196, rowY - 22, 180, 22);
    cv::rectangle(canvas, controls.cropToggleRect,
                  controls.cropEnabled ? cv::Scalar(45, 93, 70)
                                       : cv::Scalar(65, 68, 74),
                  cv::FILLED);
    cv::rectangle(canvas, controls.cropToggleRect,
                  controls.cropEnabled ? cv::Scalar(74, 190, 132)
                                       : cv::Scalar(108, 116, 126),
                  1);
    cv::putText(canvas,
                controls.cropEnabled ? "3D crop: ON (click)" :
                                       "3D crop: OFF (click)",
                cv::Point(controls.cropToggleRect.x + 8,
                          controls.cropToggleRect.y + 16),
                cv::FONT_HERSHEY_SIMPLEX, 0.39,
                cv::Scalar(230, 238, 232), 1, cv::LINE_AA);
    for (int i = 0; i < 4; ++i) {
        drawCropRange(canvas, controls, i, rowY + i * 22,
            i == 0 ? "X bounds" :
            i == 1 ? "Z bounds" :
            i == 2 ? "Height" : "Cam depth");
    }

    cv::rectangle(canvas, depthPanel, cv::Scalar(38, 43, 51), cv::FILLED);
    cv::rectangle(canvas, previewPanel, cv::Scalar(38, 43, 51), cv::FILLED);
    cv::putText(canvas, "LIVE DEPTH  |  click the turntable axis when prompted",
                cv::Point(depthPanel.x + 10, depthPanel.y - 7),
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(203, 210, 220), 1, cv::LINE_AA);
    cv::putText(canvas,     "STL OUTLINE PREVIEW  |  drag to rotate",
                cv::Point(previewPanel.x + 10, previewPanel.y - 7),
                cv::FONT_HERSHEY_SIMPLEX, 0.4,
                cv::Scalar(203, 210, 220), 1, cv::LINE_AA);

    if (!depthDisplay.empty()) {
        cv::Mat scaled;
        cv::resize(depthDisplay, scaled,
                   cv::Size(depthWidth, depthHeight), 0.0, 0.0,
                   cv::INTER_AREA);
        scaled.copyTo(canvas(controls.depthImageRect));
    } else {
        cv::putText(canvas, "Waiting for depth frames...",
                    cv::Point(depthPanel.x + 28, depthPanel.y + 100),
                    cv::FONT_HERSHEY_SIMPLEX, 0.55,
                    cv::Scalar(180, 187, 198), 1, cv::LINE_AA);
    }

    controls.cachedPreview.copyTo(canvas(controls.previewImageRect));
    if (controls.hasAxis && !depthDisplay.empty()) {
        const int markerX = controls.depthImageRect.x + cvRound(
            static_cast<double>(controls.axisPixelX) *
            std::max(0, controls.depthImageRect.width - 1) /
                depthDisplay.cols);
        const int markerY = controls.depthImageRect.y + cvRound(
            static_cast<double>(controls.axisPixelY) *
            std::max(0, controls.depthImageRect.height - 1) /
                depthDisplay.rows);
        cv::drawMarker(canvas, cv::Point(markerX, markerY),
                       cv::Scalar(40, 245, 120), cv::MARKER_CROSS, 18, 2);
    }
    if (controls.cropEnabled) {
        drawCropVolumeOverlay(canvas, controls);
    }

    const std::string labels[] = {
        controls.waitingForAxis ? "Click axis in depth view" : "Set turntable axis",
        "Capture current angle",
        "- 15 deg",
        "+ 15 deg",
        "Undo",
        "Clear scan",
        "Generate STL preview",
        "Save STL...",
        "Reset ranges",
        controls.waitingForCropCenter ? "Click target center" : "Position crop"
    };
    for (int i = 0; i < 10; ++i) {
        drawButton(canvas, controls.buttons[i], labels[i],
                   (i == 0 && controls.waitingForAxis) ||
                   (i == 9 && controls.waitingForCropCenter));
    }

    std::ostringstream scanInfo;
    scanInfo << "Turntable angle: " << controls.currentAngleDegrees
             << " deg     Captures: " << controls.captures.size()
             << "     Axis: " << (controls.hasAxis ? "calibrated" : "not set")
             << "     3D crop: " << (controls.cropEnabled ? "ON" : "OFF")
             << "     Crop center X/Z: "
             << cvRound(controls.cropCenterX) << "/"
             << cvRound(controls.cropCenterZ) << " mm";
    const int infoY = buttonY +
        (compactControls ? 2 * (buttonHeight + gap) + 8 : 60);
    cv::putText(canvas, scanInfo.str(),
                cv::Point(18, infoY), cv::FONT_HERSHEY_SIMPLEX, 0.48,
                cv::Scalar(220, 225, 232), 1, cv::LINE_AA);

    const int statusY = infoY + 24;
    drawWrappedStatus(canvas, controls.status, 18, statusY,
                      controls.canvasWidth - 36);
    const std::string help =
        "Position crop: click target center in depth view. C toggles 3D crop; "
        "R resets. Space=capture, A/D=angle, Esc=quit.";
    cv::putText(canvas, help,
                cv::Point(18, controls.canvasHeight - 15),
                cv::FONT_HERSHEY_SIMPLEX, 0.38,
                cv::Scalar(136, 145, 158), 1, cv::LINE_AA);
    return canvas;
}

bool convertDepthPoint(openni::VideoStream& stream, int x, int y,
                       openni::DepthPixel depth, float& worldX,
                       float& worldY, float& worldZ) {
    return openni::CoordinateConverter::convertDepthToWorld(
        stream, x, y, depth, &worldX, &worldY, &worldZ) == openni::STATUS_OK;
}

void drawCropVolumeOverlay(cv::Mat& canvas, const ScanControls& controls) {
    if (!controls.hasAxis || controls.stream == nullptr ||
        controls.depthWidth <= 0 || controls.depthHeight <= 0 ||
        controls.depthImageRect.width <= 0 ||
        controls.depthImageRect.height <= 0) {
        return;
    }

    cv::Point projected[8];
    bool valid[8] = {};
    const float angle = static_cast<float>(controls.currentAngleDegrees) *
        kPi / 180.0f;
    const float cosAngle = std::cos(angle);
    const float sinAngle = std::sin(angle);

    for (int corner = 0; corner < 8; ++corner) {
        const float localX = controls.cropCenterX + ((corner & 1)
            ? controls.cropBounds.maximumX : controls.cropBounds.minimumX);
        const float localZ = controls.cropCenterZ + ((corner & 2)
            ? controls.cropBounds.maximumZ : controls.cropBounds.minimumZ);
        const float height = (corner & 4)
            ? controls.cropBounds.maximumHeight
            : controls.cropBounds.minimumHeight;
        const float worldX = controls.axisX +
            localX * cosAngle + localZ * sinAngle;
        const float worldZ = controls.axisZ -
            localX * sinAngle + localZ * cosAngle;
        const float worldY = controls.axisY - height;
        int depthX = 0;
        int depthY = 0;
        openni::DepthPixel projectedDepth = 0;
        if (openni::CoordinateConverter::convertWorldToDepth(
                *controls.stream, worldX, worldY, worldZ,
                &depthX, &depthY, &projectedDepth) != openni::STATUS_OK ||
            depthX < 0 || depthX >= controls.depthWidth ||
            depthY < 0 || depthY >= controls.depthHeight) {
            continue;
        }

        projected[corner] = cv::Point(
            controls.depthImageRect.x + cvRound(
                static_cast<double>(depthX) *
                std::max(0, controls.depthImageRect.width - 1) /
                    std::max(1, controls.depthWidth - 1)),
            controls.depthImageRect.y + cvRound(
                static_cast<double>(depthY) *
                std::max(0, controls.depthImageRect.height - 1) /
                    std::max(1, controls.depthHeight - 1)));
        valid[corner] = true;
    }

    const int edges[][2] = {
        {0, 1}, {2, 3}, {4, 5}, {6, 7},
        {0, 2}, {1, 3}, {4, 6}, {5, 7},
        {0, 4}, {1, 5}, {2, 6}, {3, 7}
    };
    for (int i = 0; i < 12; ++i) {
        const int first = edges[i][0];
        const int second = edges[i][1];
        if (valid[first] && valid[second]) {
            cv::line(canvas, projected[first], projected[second],
                     cv::Scalar(57, 224, 142), 1, cv::LINE_AA);
        }
    }
}

void toggleCropFiltering(ScanControls& controls) {
    controls.cropEnabled = !controls.cropEnabled;
    controls.waitingForCropCenter = false;
    controls.mesh.clear();
    controls.previewDirty = true;
    controls.status = controls.cropEnabled
        ? "3D crop enabled. X, Z, and Height bounds will filter the scan; "
          "camera depth filtering remains active."
        : "3D crop disabled. X, Z, and Height bounds are ignored; camera "
          "depth filtering remains active.";
}

void handleMouse(int event, int x, int y, int, void* userData) {
    ScanControls& controls = *static_cast<ScanControls*>(userData);
    if (event == cv::EVENT_LBUTTONDOWN) {
        if (controls.cropToggleRect.contains(cv::Point(x, y))) {
            toggleCropFiltering(controls);
            return;
        }

        for (int i = 0; i < 10; ++i) {
            if (controls.buttons[i].contains(cv::Point(x, y))) {
                controls.action = i + ACTION_SET_AXIS;
                return;
            }
        }

        for (int i = 0; i < 4; ++i) {
            if (!controls.cropSliderTracks[i].contains(cv::Point(x, y))) {
                continue;
            }
            const int minimumX = cropRangeValueToX(
                controls.cropSliderTracks[i],
                cropRangeMinimum(controls, i),
                cropRangeGlobalMinimum(i), cropRangeGlobalMaximum(i));
            const int maximumX = cropRangeValueToX(
                controls.cropSliderTracks[i],
                cropRangeMaximum(controls, i),
                cropRangeGlobalMinimum(i), cropRangeGlobalMaximum(i));
            const bool setMinimum = std::abs(x - minimumX) <=
                std::abs(x - maximumX);
            controls.draggingCropHandle = i * 2 + (setMinimum ? 0 : 1);
            const float selectedValue = cropRangeValueAtX(
                controls.cropSliderTracks[i], x,
                cropRangeGlobalMinimum(i), cropRangeGlobalMaximum(i));
            setCropRangeValue(controls, i, setMinimum,
                              static_cast<int>(std::lround(selectedValue)));
            controls.mesh.clear();
            controls.previewDirty = true;
            controls.status =
                "Crop settings changed. Generate the preview again to apply "
                "them to all captures.";
            return;
        }

        if (controls.waitingForAxis &&
            controls.depthImageRect.contains(cv::Point(x, y)) &&
            controls.stream != nullptr && controls.depthWidth > 0 &&
            controls.depthHeight > 0 &&
            controls.latestDepth.size() ==
                static_cast<std::size_t>(controls.depthWidth) *
                    controls.depthHeight) {
            const int pixelX = std::max(0, std::min(
                controls.depthWidth - 1,
                (x - controls.depthImageRect.x) * controls.depthWidth /
                    controls.depthImageRect.width));
            const int pixelY = std::max(0, std::min(
                controls.depthHeight - 1,
                (y - controls.depthImageRect.y) * controls.depthHeight /
                    controls.depthImageRect.height));
            const openni::DepthPixel depth = controls.latestDepth[
                static_cast<std::size_t>(pixelY) * controls.depthWidth +
                pixelX];
            if (depth == 0) {
                controls.status =
                    "That pixel has no depth. Click a valid point on the "
                    "turntable surface.";
                return;
            }
            if (!convertDepthPoint(*controls.stream, pixelX, pixelY, depth,
                                   controls.axisX, controls.axisY,
                                   controls.axisZ)) {
                controls.status = "Could not calibrate the axis at that pixel.";
                return;
            }
            controls.hasAxis = true;
            controls.axisPixelX = pixelX;
            controls.axisPixelY = pixelY;
            controls.waitingForAxis = false;
            controls.status =
                "Axis calibrated. Rotate the turntable to 0 degrees and "
                "capture the first view.";
            return;
        }

        if (controls.waitingForCropCenter &&
            controls.depthImageRect.contains(cv::Point(x, y)) &&
            controls.stream != nullptr && controls.hasAxis &&
            controls.depthWidth > 0 && controls.depthHeight > 0 &&
            controls.latestDepth.size() ==
                static_cast<std::size_t>(controls.depthWidth) *
                    controls.depthHeight) {
            const int pixelX = std::max(0, std::min(
                controls.depthWidth - 1,
                (x - controls.depthImageRect.x) * controls.depthWidth /
                    controls.depthImageRect.width));
            const int pixelY = std::max(0, std::min(
                controls.depthHeight - 1,
                (y - controls.depthImageRect.y) * controls.depthHeight /
                    controls.depthImageRect.height));
            const openni::DepthPixel depth = controls.latestDepth[
                static_cast<std::size_t>(pixelY) * controls.depthWidth +
                pixelX];
            float worldX = 0.0f;
            float worldY = 0.0f;
            float worldZ = 0.0f;
            if (depth == 0 ||
                !convertDepthPoint(*controls.stream, pixelX, pixelY, depth,
                                   worldX, worldY, worldZ)) {
                controls.status =
                    "That pixel has no valid depth. Click the target center.";
                return;
            }

            const float relativeX = worldX - controls.axisX;
            const float relativeZ = worldZ - controls.axisZ;
            const float angle =
                static_cast<float>(controls.currentAngleDegrees) *
                kPi / 180.0f;
            controls.cropCenterX =
                relativeX * std::cos(angle) - relativeZ * std::sin(angle);
            controls.cropCenterZ =
                relativeX * std::sin(angle) + relativeZ * std::cos(angle);
            controls.waitingForCropCenter = false;
            controls.mesh.clear();
            controls.previewDirty = true;
            controls.status =
                "Crop box moved to the selected target center. Adjust its "
                "size and regenerate the preview.";
            return;
        }

        if (controls.previewImageRect.contains(cv::Point(x, y))) {
            controls.draggingPreview = true;
            controls.previousMouseX = x;
            controls.previousMouseY = y;
        }
    } else if (event == cv::EVENT_MOUSEMOVE &&
               controls.draggingCropHandle >= 0) {
        const int rangeIndex = controls.draggingCropHandle / 2;
        const bool setMinimum = controls.draggingCropHandle % 2 == 0;
        const float selectedValue = cropRangeValueAtX(
            controls.cropSliderTracks[rangeIndex], x,
            cropRangeGlobalMinimum(rangeIndex),
            cropRangeGlobalMaximum(rangeIndex));
        setCropRangeValue(controls, rangeIndex, setMinimum,
                          static_cast<int>(std::lround(selectedValue)));
        controls.mesh.clear();
        controls.previewDirty = true;
        controls.status =
            "Crop settings changed. Generate the preview again to apply "
            "them to all captures.";
    } else if (event == cv::EVENT_MOUSEMOVE && controls.draggingPreview) {
        controls.previewYaw += (x - controls.previousMouseX) * 0.01f;
        controls.previewPitch = std::max(
            -1.35f, std::min(1.35f,
                controls.previewPitch +
                    (y - controls.previousMouseY) * 0.01f));
        controls.previewDirty = true;
        controls.previousMouseX = x;
        controls.previousMouseY = y;
    } else if (event == cv::EVENT_LBUTTONUP) {
        controls.draggingCropHandle = -1;
        controls.draggingPreview = false;
    }
}

int normalizedAngle(int angle) {
    int normalized = angle % 360;
    if (normalized < 0) {
        normalized += 360;
    }
    return normalized;
}

void clearGeneratedMesh(ScanControls& controls) {
    controls.mesh.clear();
    controls.previewDirty = true;
}

void resetCropRanges(ScanControls& controls) {
    controls.cropCenterX = 0.0f;
    controls.cropCenterZ = 0.0f;
    controls.waitingForCropCenter = false;
    controls.cropBounds.minimumX = -180.0f;
    controls.cropBounds.maximumX = 180.0f;
    controls.cropBounds.minimumZ = -180.0f;
    controls.cropBounds.maximumZ = 180.0f;
    controls.cropBounds.minimumHeight = 0.0f;
    controls.cropBounds.maximumHeight = kMaximumHeightMm;
    controls.minimumCameraDepthMm = 500;
    controls.maximumCameraDepthMm = 3500;
    clearGeneratedMesh(controls);
    controls.status =
        "Crop bounds and camera depth range reset to their defaults.";
}

void captureCurrentDepth(ScanControls& controls) {
    if (!controls.hasAxis) {
        controls.status = "Set the turntable axis before capturing.";
        return;
    }
    if (controls.latestDepth.empty()) {
        controls.status = "Waiting for a valid depth frame.";
        return;
    }

    const int angle = normalizedAngle(controls.currentAngleDegrees);
    for (std::size_t i = 0; i < controls.captures.size(); ++i) {
        if (normalizedAngle(controls.captures[i].angleDegrees) == angle) {
            controls.status = "Angle " + std::to_string(angle) +
                " deg is already captured. Choose another angle or undo.";
            return;
        }
    }

    CapturedDepthFrame frame;
    frame.depth = controls.latestDepth;
    frame.width = controls.depthWidth;
    frame.height = controls.depthHeight;
    frame.angleDegrees = controls.currentAngleDegrees;
    controls.captures.push_back(frame);
    clearGeneratedMesh(controls);

    std::ostringstream message;
    message << "Captured " << angle << " deg. Turn the platform "
            << kTurntableStepDegrees
            << " deg in the same direction, then advance the angle.";
    controls.status = message.str();
}

void generateMesh(ScanControls& controls) {
    if (!controls.hasAxis) {
        controls.status = "Calibrate the turntable axis before scanning.";
        return;
    }

    std::vector<int> captureAngles;
    captureAngles.reserve(controls.captures.size());
    for (std::size_t i = 0; i < controls.captures.size(); ++i) {
        captureAngles.push_back(controls.captures[i].angleDegrees);
    }
    std::string coverageError;
    if (!scanner::validateTurntableCoverage(captureAngles, 30,
                                            coverageError)) {
        controls.mesh.clear();
        controls.previewDirty = true;
        controls.status = coverageError;
        return;
    }

    controls.status = "Generating mesh from captured depth frames...";
    cv::Mat waitingCanvas = makeCanvas(
        makeDepthDisplay(controls.latestDepth, controls.depthWidth,
                         controls.depthHeight,
                         controls.minimumCameraDepthMm,
                         controls.maximumCameraDepthMm),
        controls);
    cv::imshow("PrimeSense 360 Scan", waitingCanvas);
    cv::waitKey(1);

    std::vector<scanner::PolarSample> samples;
    std::size_t conversionFailures = 0;
    const int pixelStride = 2;
    for (std::size_t capture = 0; capture < controls.captures.size();
         ++capture) {
        const CapturedDepthFrame& frame = controls.captures[capture];
        const float angle = static_cast<float>(frame.angleDegrees) *
            kPi / 180.0f;
        const float cosAngle = std::cos(angle);
        const float sinAngle = std::sin(angle);

        for (int y = 0; y < frame.height; y += pixelStride) {
            for (int x = 0; x < frame.width; x += pixelStride) {
                const openni::DepthPixel depth = frame.depth[
                    static_cast<std::size_t>(y) * frame.width + x];
                if (depth < controls.minimumCameraDepthMm ||
                    depth > controls.maximumCameraDepthMm) {
                    continue;
                }

                float worldX = 0.0f;
                float worldY = 0.0f;
                float worldZ = 0.0f;
                if (!convertDepthPoint(*controls.stream, x, y, depth,
                                       worldX, worldY, worldZ)) {
                    ++conversionFailures;
                    continue;
                }

                const float relativeX = worldX - controls.axisX;
                const float relativeZ = worldZ - controls.axisZ;
                const float objectX =
                    relativeX * cosAngle - relativeZ * sinAngle;
                const float objectZ =
                    relativeX * sinAngle + relativeZ * cosAngle;
                const float cropX = controls.cropEnabled
                    ? objectX - controls.cropCenterX : objectX;
                const float cropZ = controls.cropEnabled
                    ? objectZ - controls.cropCenterZ : objectZ;
                const float radius = std::sqrt(cropX * cropX + cropZ * cropZ);
                const float height = controls.axisY - worldY;
                if (radius > kMaximumRadiusMm ||
                    (controls.cropEnabled &&
                     !scanner::contains(controls.cropBounds, cropX, height,
                                        cropZ))) {
                    continue;
                }

                scanner::PolarSample sample;
                sample.angleDegrees = std::atan2(cropX, cropZ) *
                    180.0f / kPi;
                sample.heightMm = height;
                sample.radiusMm = radius;
                samples.push_back(sample);
            }
        }
    }

    if (conversionFailures > 0) {
        std::cerr << "OpenNI coordinate conversion failed for "
                  << conversionFailures << " captured depth samples."
                  << std::endl;
        controls.mesh.clear();
        controls.status =
            "Coordinate conversion failed for " +
            std::to_string(conversionFailures) +
            " samples; see the console. No mesh was generated.";
        return;
    }

    std::string error;
    if (!scanner::buildRadialMesh(samples, kMaximumRadiusMm,
                                  kMaximumHeightMm, controls.mesh, error)) {
        controls.mesh.clear();
        controls.previewDirty = true;
        controls.status = "Mesh generation failed: " + error;
        return;
    }

    controls.status = "STL preview generated. Inspect it, then use Save STL.";
    controls.previewDirty = true;
}

bool saveStl(ScanControls& controls) {
    if (controls.mesh.empty()) {
        controls.status = "Generate a mesh preview before saving an STL.";
        return false;
    }

    char filename[MAX_PATH] = "scan.stl";
    OPENFILENAMEA dialog = {};
    dialog.lStructSize = sizeof(dialog);
    dialog.lpstrFilter = "STL mesh (*.stl)\0*.stl\0All files (*.*)\0*.*\0";
    dialog.lpstrFile = filename;
    dialog.nMaxFile = sizeof(filename);
    dialog.lpstrDefExt = "stl";
    dialog.Flags = OFN_PATHMUSTEXIST | OFN_OVERWRITEPROMPT;
    if (!GetSaveFileNameA(&dialog)) {
        const DWORD error = CommDlgExtendedError();
        if (error != 0) {
            controls.status = "The Save dialog failed (code " +
                std::to_string(error) + ").";
            return false;
        }
        controls.status = "Save cancelled.";
        return false;
    }

    std::string error;
    if (!scanner::writeBinaryStl(controls.mesh, filename, error)) {
        controls.status = "Could not save STL: " + error;
        return false;
    }
    controls.status = "Saved " + std::string(filename) + " (millimeters).";
    return true;
}

void processAction(ScanControls& controls, int action) {
    switch (action) {
    case ACTION_SET_AXIS:
        if (!controls.captures.empty()) {
            controls.status =
                "Clear the current scan before recalibrating the turntable axis.";
            break;
        }
        controls.waitingForAxis = true;
        controls.waitingForCropCenter = false;
        controls.hasAxis = false;
        controls.cropCenterX = 0.0f;
        controls.cropCenterZ = 0.0f;
        controls.status =
            "Click the rotation center at the object's base (not the "
            "turntable edge).";
        break;
    case ACTION_CAPTURE:
        captureCurrentDepth(controls);
        break;
    case ACTION_ANGLE_BACK:
        controls.currentAngleDegrees -= kTurntableStepDegrees;
        controls.status = "Rotate the platform to " +
            std::to_string(normalizedAngle(controls.currentAngleDegrees)) +
            " deg, then capture.";
        break;
    case ACTION_ANGLE_FORWARD:
        controls.currentAngleDegrees += kTurntableStepDegrees;
        controls.status = "Rotate the platform to " +
            std::to_string(normalizedAngle(controls.currentAngleDegrees)) +
            " deg, then capture.";
        break;
    case ACTION_UNDO:
        if (controls.captures.empty()) {
            controls.status = "There are no captures to undo.";
        } else {
            controls.captures.pop_back();
            clearGeneratedMesh(controls);
            controls.status = "Removed the most recent depth capture.";
        }
        break;
    case ACTION_CLEAR:
        controls.captures.clear();
        clearGeneratedMesh(controls);
        controls.currentAngleDegrees = 0;
        controls.status = "Scan cleared. Axis calibration was kept.";
        break;
    case ACTION_GENERATE:
        generateMesh(controls);
        break;
    case ACTION_SAVE:
        saveStl(controls);
        break;
    case ACTION_RESET_RANGES:
        resetCropRanges(controls);
        break;
    case ACTION_POSITION_CROP:
        if (!controls.hasAxis) {
            controls.status =
                "Set the turntable axis before positioning the crop box.";
            break;
        }
        controls.cropEnabled = true;
        controls.waitingForAxis = false;
        controls.waitingForCropCenter = true;
        controls.status =
            "Click the target center in the live depth view. The crop box "
            "will move there and follow the turntable.";
        break;
    default:
        break;
    }
}

bool windowIsOpen(const std::string& windowName) {
    return cv::getWindowProperty(windowName, cv::WND_PROP_VISIBLE) > 0;
}

} // namespace

int main() {
    openni::Status status = openni::OpenNI::initialize();
    if (status != openni::STATUS_OK) {
        std::cerr << "Could not initialize OpenNI: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        return 1;
    }

    openni::Device device;
    status = device.open(openni::ANY_DEVICE);
    if (status != openni::STATUS_OK) {
        std::cerr << "Could not open the PrimeSense camera: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        openni::OpenNI::shutdown();
        return 1;
    }

    openni::VideoStream depthStream;
    status = depthStream.create(device, openni::SENSOR_DEPTH);
    if (status != openni::STATUS_OK) {
        std::cerr << "Could not create the depth stream: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        device.close();
        openni::OpenNI::shutdown();
        return 1;
    }
    if (!selectDepthMode(depthStream)) {
        depthStream.destroy();
        device.close();
        openni::OpenNI::shutdown();
        return 1;
    }
    status = depthStream.start();
    if (status != openni::STATUS_OK) {
        std::cerr << "Could not start the depth stream: "
                  << openni::OpenNI::getExtendedError() << std::endl;
        depthStream.destroy();
        device.close();
        openni::OpenNI::shutdown();
        return 1;
    }

    const openni::VideoMode mode = depthStream.getVideoMode();
    std::cout << "PrimeSense scanner started at "
              << mode.getResolutionX() << "x" << mode.getResolutionY()
              << ". Set the turntable axis, capture 15-degree steps, and "
                 "generate the STL in the preview."
              << std::endl;

    ScanControls controls;
    controls.stream = &depthStream;
    const std::string windowName = "PrimeSense 360 Scan";
    cv::namedWindow(windowName, cv::WINDOW_AUTOSIZE);
    cv::setMouseCallback(windowName, handleMouse, &controls);

    bool running = true;
    while (running) {
        openni::VideoStream* streams[] = {&depthStream};
        int readyStreamIndex = 0;
        status = openni::OpenNI::waitForAnyStream(
            streams, 1, &readyStreamIndex, 40);
        if (status == openni::STATUS_OK) {
            openni::VideoFrameRef frame;
            status = depthStream.readFrame(&frame);
            if (status != openni::STATUS_OK || !frame.isValid()) {
                std::cerr << "Could not read the depth frame: "
                          << openni::OpenNI::getExtendedError() << std::endl;
                controls.status = "Camera frame read failed; see console.";
                break;
            }

            controls.depthWidth = frame.getWidth();
            controls.depthHeight = frame.getHeight();
            cv::Mat depthMat(
                controls.depthHeight, controls.depthWidth, CV_16UC1,
                const_cast<void*>(frame.getData()), frame.getStrideInBytes());
            depthMat = depthMat.clone();
            controls.latestDepth.resize(
                static_cast<std::size_t>(controls.depthWidth) *
                controls.depthHeight);
            for (int y = 0; y < controls.depthHeight; ++y) {
                const openni::DepthPixel* row =
                    depthMat.ptr<openni::DepthPixel>(y);
                std::copy(row, row + controls.depthWidth,
                    controls.latestDepth.begin() +
                        static_cast<std::size_t>(y) * controls.depthWidth);
            }
        } else if (status != openni::STATUS_TIME_OUT) {
            std::cerr << "Waiting for a depth frame failed: "
                      << openni::OpenNI::getExtendedError() << std::endl;
            controls.status = "Camera stream failed; see console.";
            break;
        }

        cv::Mat depthDisplay = makeDepthDisplay(
            controls.latestDepth, controls.depthWidth, controls.depthHeight,
            controls.minimumCameraDepthMm, controls.maximumCameraDepthMm);
        cv::Mat canvas = makeCanvas(depthDisplay, controls);
        cv::imshow(windowName, canvas);

        const int key = cv::waitKey(1) & 0xFF;
        if (key == 27 || key == 'q' || key == 'Q') {
            running = false;
        } else if (key == ' ') {
            processAction(controls, ACTION_CAPTURE);
        } else if (key == 'a' || key == 'A') {
            processAction(controls, ACTION_ANGLE_BACK);
        } else if (key == 'd' || key == 'D') {
            processAction(controls, ACTION_ANGLE_FORWARD);
        } else if (key == 'r' || key == 'R') {
            resetCropRanges(controls);
        } else if (key == 'c' || key == 'C') {
            toggleCropFiltering(controls);
        }

        const int action = controls.action;
        controls.action = ACTION_NONE;
        if (action != ACTION_NONE) {
            processAction(controls, action);
        }
        if (!windowIsOpen(windowName)) {
            running = false;
        }
    }

    depthStream.stop();
    depthStream.destroy();
    device.close();
    openni::OpenNI::shutdown();
    cv::destroyAllWindows();
    return 0;
}
