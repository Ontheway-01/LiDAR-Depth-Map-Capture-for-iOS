//
//  OpenCVWrapper.m
//  DepthCamera
//
//  Created by 이은화 on 5/20/25.
//

#import "OpenCVWrapper.h"
#undef NO
#import <opencv2/opencv.hpp>
#import <ARKit/ARKit.h>
#import <CoreVideo/CoreVideo.h>
#import <opencv2/imgcodecs/ios.h>
#import <opencv2/calib3d.hpp>
#include <opencv2/core/quaternion.hpp>

@implementation OpenCVWrapper

//from here triangle's vertex: red, green, blue
// MARK: - 호모그래피 분해를 통한 회전 계산
+ (NSDictionary *)computeRotationViaHomography:(const std::vector<cv::Point2f>&)imagePoints
                                 objectPoints:(const std::vector<cv::Point3f>&)objectPoints
                               cameraMatrix:(const cv::Mat&)cameraMatrix {
    // 호모그래피 계산 (Z=0 평면 가정)
    std::vector<cv::Point2f> objectPoints2D;
    for (const auto& pt : objectPoints) {
        objectPoints2D.emplace_back(pt.x, pt.y);
    }
    cv::Mat H = cv::findHomography(objectPoints2D, imagePoints);
    
    // 호모그래피 분해
    std::vector<cv::Mat> Rs, Ts, normals;
    cv::decomposeHomographyMat(H, cameraMatrix, Rs, Ts, normals);
    
    // 첫 번째 해를 선택 (실제 환경에 맞게 보정 필요)
    cv::Mat R;
    if (!Rs.empty()) {
        R = Rs[0];
    } else {
        R = cv::Mat::eye(3, 3, CV_64F);
    }
    
    return [self rotationMatrixToDict:R];
}

// MARK: - 소실점 분석을 통한 회전 계산
+ (NSDictionary *)computeRotationViaVanishingPoints:(const std::vector<cv::Point2f>&)imagePoints
                                     cameraMatrix:(const cv::Mat&)cameraMatrix {
    // 삼각형 에지에서 소실점 계산
    std::vector<cv::Vec4i> lines;
    for (int i=0; i<3; i++) {
        cv::Point2f p1 = imagePoints[i];
        cv::Point2f p2 = imagePoints[(i+1)%3];
        lines.emplace_back(p1.x, p1.y, p2.x, p2.y);
    }
    
    // 소실점 계산 (두 에지의 교차점)
    cv::Point2f vp1 = computeVanishingPoint(lines[0], lines[1]);
    cv::Point2f vp2 = computeVanishingPoint(lines[1], lines[2]);
    
    // 소실점을 이용한 회전 계산
    double fx = cameraMatrix.at<double>(0,0);
    double fy = cameraMatrix.at<double>(1,1);
    double cx = cameraMatrix.at<double>(0,2);
    double cy = cameraMatrix.at<double>(1,2);
    
    double pitch = atan2(vp1.y - cy, fy);
    double yaw = atan2(vp1.x - cx, fx);
    
    cv::Mat R = eulerAnglesToRotationMatrix(pitch, 0, yaw);
    return [self rotationMatrixToDict:R];
}

// MARK: - LiDAR 융합을 통한 회전 계산
+ (NSDictionary *)computeRotationViaLiDAR:(const std::vector<cv::Point3f>&)lidarPoints
                            objectPoints:(const std::vector<cv::Point3f>&)objectPoints {
    cv::Mat R, t;
    rigid_transform_3D(lidarPoints, objectPoints, R, t);
    return [self rotationMatrixToDict:R];
}

// MARK: - 공통 함수
+ (NSDictionary *)rotationMatrixToDict:(const cv::Mat&)R {
    NSMutableDictionary *dict = [NSMutableDictionary dictionary];
    for (int i=0; i<3; i++) {
        for (int j=0; j<3; j++) {
            NSString *key = [NSString stringWithFormat:@"r%d%d", i, j];
            dict[key] = @(R.at<double>(i, j));
        }
    }
    return dict;
}

cv::Point2f computeVanishingPoint(const cv::Vec4i& line1, const cv::Vec4i& line2) {
    // 두 직선의 교차점 계산
    cv::Point2f p1(line1[0], line1[1]);
    cv::Point2f p2(line1[2], line1[3]);
    cv::Point2f q1(line2[0], line2[1]);
    cv::Point2f q2(line2[2], line2[3]);
    
    cv::Point2f vp;
    float denom = (p1.x - p2.x)*(q1.y - q2.y) - (p1.y - p2.y)*(q1.x - q2.x);
    if (denom != 0) {
        vp.x = ((p1.x*p2.y - p1.y*p2.x)*(q1.x - q2.x) - (p1.x - p2.x)*(q1.x*q2.y - q1.y*q2.x)) / denom;
        vp.y = ((p1.x*p2.y - p1.y*p2.x)*(q1.y - q2.y) - (p1.y - p2.y)*(q1.x*q2.y - q1.y*q2.x)) / denom;
    }
    return vp;
}

cv::Mat eulerAnglesToRotationMatrix(double pitch, double roll, double yaw) {
    cv::Mat R_x = (cv::Mat_<double>(3,3) <<
        1, 0, 0,
        0, cos(pitch), -sin(pitch),
        0, sin(pitch), cos(pitch));
    
    cv::Mat R_y = (cv::Mat_<double>(3,3) <<
        cos(roll), 0, sin(roll),
        0, 1, 0,
        -sin(roll), 0, cos(roll));
        
    cv::Mat R_z = (cv::Mat_<double>(3,3) <<
        cos(yaw), -sin(yaw), 0,
        sin(yaw), cos(yaw), 0,
        0, 0, 1);
        
    return R_z * R_y * R_x;
}

+ (NSDictionary*)detectTriangleAndComputePoseWithPixelBuffer:
(CVPixelBufferRef)pixelBuffer
                                                 depthBuffer:
(CVPixelBufferRef)depthBuffer
                                             cameraIntrinsic:
(const float *)intrinsics
{
    NSDictionary *result = {};

    // 1. pixelBuffer(YUV) → BGR 변환 (OpenCV)
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    size_t width = CVPixelBufferGetWidth(pixelBuffer);
    size_t height = CVPixelBufferGetHeight(pixelBuffer);

    // Y plane
    uint8_t *yPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
    size_t yPitch = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
    cv::Mat yMat((int)height, (int)width, CV_8UC1, yPlane, yPitch);

    // UV plane (NV12)
    uint8_t *uvPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1);
    size_t uvPitch = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
    cv::Mat uvMat((int)height/2, (int)width/2, CV_8UC2, uvPlane, uvPitch);

    // YUV → BGR
    cv::Mat bgrMat;
    cv::cvtColorTwoPlane(yMat, uvMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);

    // 2. HSV 변환 및 색상별 마스크
    cv::Mat hsvMat;
    cv::cvtColor(bgrMat, hsvMat, cv::COLOR_BGR2HSV);

    // 빨강 (0-10 & 160-180)
    cv::Mat lowerRed1, lowerRed2, redMask;
    cv::inRange(hsvMat, cv::Scalar(0, 150, 100), cv::Scalar(10, 255, 255), lowerRed1);
    cv::inRange(hsvMat, cv::Scalar(160, 150, 100), cv::Scalar(179, 255, 255), lowerRed2);
    cv::bitwise_or(lowerRed1, lowerRed2, redMask);

    // 초록 (35-85)
    cv::Mat greenMask;
    cv::inRange(hsvMat, cv::Scalar(35, 100, 100), cv::Scalar(85, 255, 255), greenMask);

    // 파랑 (100-130)
    cv::Mat blueMask;
    cv::inRange(hsvMat, cv::Scalar(100, 100, 100), cv::Scalar(130, 255, 255), blueMask);

    // 3. 블러 및 원 검출
    cv::GaussianBlur(redMask, redMask, cv::Size(9,9), 2);
    cv::GaussianBlur(greenMask, greenMask, cv::Size(9,9), 2);
    cv::GaussianBlur(blueMask, blueMask, cv::Size(9,9), 2);

    std::vector<cv::Vec3f> redCircles, greenCircles, blueCircles;
    cv::HoughCircles(redMask, redCircles, cv::HOUGH_GRADIENT, 1, redMask.rows/8, 100, 20, 0, 0);
    cv::HoughCircles(greenMask, greenCircles, cv::HOUGH_GRADIENT, 1, greenMask.rows/8, 100, 20, 0, 0);
    cv::HoughCircles(blueMask, blueCircles, cv::HOUGH_GRADIENT, 1, blueMask.rows/8, 100, 20, 0, 0);

    // 4. 검출 실패 시 기본값 반환
    if (redCircles.empty() || greenCircles.empty() || blueCircles.empty()) {
        printf("Less than 3 circles detected.\n");
        NSMutableArray *pixelCenters = [NSMutableArray array];
        for (int i = 0; i < 3; i++) {
            [pixelCenters addObject:@{@"x": @(0.0f), @"y": @(0.0f), @"radius": @(0.0f)}];
        }
        NSMutableDictionary *rotationDict = [NSMutableDictionary dictionary];
        for (int i = 0; i < 3; i++)
            for (int j = 0; j < 3; j++)
                rotationDict[[NSString stringWithFormat:@"r%d%d", i, j]] = @(0.0f);

        return @{
            @"camera_position": @{@"x": @(0.0f), @"y": @(0.0f), @"z": @(0.0f)},
            @"lidar_position": @{@"x": @(0.0f), @"y": @(0.0f), @"z": @(0.0f)},
            @"rotation_world_to_camera": rotationDict,
            @"pixel_centers": pixelCenters
        };
    }

    // 5. 깊이 버퍼에서 3D 좌표 추출
    CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
    float *depthBase = (float *)CVPixelBufferGetBaseAddress(depthBuffer);
    size_t depthWidth = CVPixelBufferGetWidth(depthBuffer);
    size_t depthHeight = CVPixelBufferGetHeight(depthBuffer);
    int depthBytesPerRow = CVPixelBufferGetBytesPerRow(depthBuffer) / sizeof(float);
    cv::Mat depthMat((int)depthHeight, (int)depthWidth, CV_32FC1, depthBase, depthBytesPerRow * sizeof(float));
    CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);

    // 6. LiDAR 3D 좌표 계산 (빨강, 초록, 파랑 순)
    std::vector<cv::Vec3f> detectedCircles = {redCircles[0], greenCircles[0], blueCircles[0]};
    
    struct CircleInfo {
        float x, y, radius, depth;
    };
    std::vector<CircleInfo> points2DDepth;
    NSMutableArray *pixelCenters = [NSMutableArray array];
    for (int i = 0; i < 3; i++) {
        float x = detectedCircles[i][0];
        float y = detectedCircles[i][1];
        float radius = detectedCircles[i][2];
        [pixelCenters addObject:@{@"x": @(x), @"y": @(y), @"radius": @(radius)}];
        
        int dx = (int)round(x * ((float)depthWidth / width));
        int dy = (int)round(y * ((float)depthHeight / height));
        
        float depth = 0.0f;
        if (dx >= 0 && dx < depthWidth && dy >= 0 && dy < depthHeight) {
            depth = depthMat.at<float>(dy, dx);
        }
        
        points2DDepth.push_back({x, y, radius, depth * 100.0f});
    }
    std::vector<cv::Point3f> measuredPts;

    cv::Matx33f K(
        intrinsics[0], intrinsics[3], intrinsics[6],
        intrinsics[1], intrinsics[4], intrinsics[7],
        intrinsics[2], intrinsics[5], intrinsics[8]
    );
    float fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);

    for (int i = 0; i < 3; i++) {
        float u = points2DDepth[i].x;
        float v = points2DDepth[i].y;
//        float Z = points2DDepth[idx].depth;
        
        float denominator = sqrt( pow((u - cx)/fx, 2) + pow((v - cy)/fy, 2) + 1 );
        float Z = points2DDepth[i].depth / denominator;

        float X = - (v - cy) / fy * Z;        // X: 부호 그대로
        float Y = (u - cx) / fx * Z;
        measuredPts.push_back(cv::Point3f(X, Y, Z));
    }

    // 7. 기준 삼각형 좌표 (빨강, 초록, 파랑 순)
    std::vector<cv::Point3f> triPts = {
        {0.0f, 0.0f, 0.0f},
        {4.0f, 0.0f, 0.0f},
        {2.0f, 2.0f * std::sqrt(3.0f), 0.0f}
    };

    // 8. LiDAR 위치 계산 (rigid transform)
    cv::Mat R_lidar, t_lidar;
    rigid_transform_3D(measuredPts, triPts, R_lidar, t_lidar);

    // 9. 카메라 포즈 계산 (solvePnP)
    std::vector<cv::Point2f> imagePoints = {
        cv::Point2f(detectedCircles[0][0], detectedCircles[0][1]),
        cv::Point2f(detectedCircles[1][0], detectedCircles[1][1]),
        cv::Point2f(detectedCircles[2][0], detectedCircles[2][1])
    };
    cv::Mat cameraMatrix = (cv::Mat_<double>(3,3) <<
        intrinsics[0], intrinsics[3], intrinsics[6],
        intrinsics[1], intrinsics[4], intrinsics[7],
        intrinsics[2], intrinsics[5], intrinsics[8]);
    std::vector<cv::Point3d> triPts3D = {
        {0.0f, 0.0f, 0.0f},
        {4.0f, 0.0f, 0.0f},
        {2.0f, 2.0f * std::sqrt(3.0f), 0.0f}
    };
    std::vector<cv::Point2d> imagePoints2D = {
        cv::Point2d(detectedCircles[0][0], detectedCircles[0][1]),
        cv::Point2d(detectedCircles[1][0], detectedCircles[1][1]),
        cv::Point2d(detectedCircles[2][0], detectedCircles[2][1])
    };
    std::vector<cv::Mat> rvecs, tvecs;
    cv::Mat rvec, tvec; // 이동 벡터 초기값 (0)
    cv::solveP3P(triPts, imagePoints, cameraMatrix, cv::noArray(), rvecs, tvecs, cv::SOLVEPNP_P3P);
    if (rvecs.empty() || tvecs.empty()){
        printf("failed");
        NSMutableArray *pixelCenterss = [NSMutableArray array];
        for (int i = 0; i < 3; i++) {
            [pixelCenterss addObject:@{@"x": @(0.0f), @"y": @(0.0f), @"radius": @(0.0f)}];
        }
        return @{
            @"lidar_position": @{
                @"x": @(0.0f),
                @"y": @(0.0f),
                @"z": @(0.0f)
            },
            @"normal": @{ @"x": @(0.0f), @"y": @(0.0f), @"z": @(0.0f) },
            @"plane_equation": @{ @"a": @(0.0f), @"b": @(0.0f), @"c": @(0.0f), @"d": @(0.0f) },
            @"pixel_centers": pixelCenterss
        };
    }
    cv::Point3f lidarPos(t_lidar.at<double>(0,0), t_lidar.at<double>(1,0), t_lidar.at<double>(2,0));
    cv::Point3f vertex1(0.0f, 0.0f, 0.0f);
    double lidarDepth = cv::norm(lidarPos - vertex1);

    if (!rvecs.empty()) {
        int bestIdx = -1;
        double minDiff = std::numeric_limits<double>::max();
        for (size_t i = 0; i < tvecs.size(); ++i) {
            // 카메라 중심(=tvec) ~ 월드 원점(=objectPoints[0])의 유클리드 거리
            cv::Mat tvec = tvecs[i];
            cv::Point3f cameraPos(tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2));
            double distance = cv::norm(cameraPos);
            double diff = std::abs(distance - lidarDepth);
            if (diff < minDiff) {
                minDiff = diff;
                bestIdx = i;
            }
        }
        // bestIdx가 최적 해
        cv::Mat rvec = rvecs[bestIdx];
        cv::Mat tvec = tvecs[bestIdx];
    }
    cv::Mat R;
    cv::Rodrigues(rvec, R); // rvec → 회전 행렬 R 변환

    // 회전 행렬 R의 세 번째 열이 카메라의 Z축 (정면 방향)
    cv::Mat normal = (cv::Mat_<double>(3,1) << R.at<double>(0,2),
                                              R.at<double>(1,2),
                                              R.at<double>(2,2));
    // 카메라 위치 (tvec)에서 평면 방정식의 D 계산
    double A = normal.at<double>(0);
    double B = normal.at<double>(1);
    double C = normal.at<double>(2);
    double D = -(A * tvec.at<double>(0) +
                 B * tvec.at<double>(1) +
                 C * tvec.at<double>(2));
    printf("A: %3f, B: %3f, C: %3f, D: %3f", A, B, C, D);
    
    // 11. 결과 반환
    return @{
        @"lidar_position": @{
            @"x": @(t_lidar.at<double>(0,0)),
            @"y": @(t_lidar.at<double>(1,0)),
            @"z": @(t_lidar.at<double>(2,0))
        },
        @"normal": @{ @"x": @(A), @"y": @(B), @"z": @(C) },
        @"plane_equation": @{ @"a": @(A), @"b": @(B), @"c": @(C), @"d": @(D) },
//        @"rotation_world_to_camera": rotationDict,
        @"pixel_centers": pixelCenters
    };
}

//from here triangle's vertex: red
+ (NSArray<NSDictionary *> *)detectTriangleRedCirclesInPixelBuffer:
(CVPixelBufferRef)pixelBuffer
                                               depthBuffer:
(CVPixelBufferRef)depthBuffer
                                           intrinsicsArray:
(const float *)intrinsicsArray
                                              cameraMatrix:
(const float *)cameraMatrix
                                           lidarWorldArray:
(const float *)lidarWorldArray
{
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    size_t rgbWidth = CVPixelBufferGetWidth(pixelBuffer);
    size_t rgbHeight = CVPixelBufferGetHeight(pixelBuffer);
    
    printf("width: %zu", rgbWidth);
    printf("height: %zu", rgbHeight);
    
    uint8_t *yPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
    uint8_t *uvPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1);
    
    size_t yPitch = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
    size_t uvPitch = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
    
    cv::Mat yMat((int)rgbHeight, (int)rgbWidth, CV_8UC1, yPlane, yPitch);
    cv::Mat uvMat((int)rgbHeight/2, (int)rgbWidth/2, CV_8UC2, uvPlane, uvPitch);
    
    cv::Mat yuvMat((int)(rgbHeight * 3 / 2), (int)rgbWidth, CV_8UC1);
    memcpy(yuvMat.data, yMat.data, rgbWidth * rgbHeight);
    memcpy(yuvMat.data + rgbWidth * rgbHeight, uvMat.data, rgbWidth * rgbHeight / 2);
    
    cv::Mat bgrMat;
    cv::cvtColor(yuvMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    cv::Mat hsvMat;
    cv::cvtColor(bgrMat, hsvMat, cv::COLOR_BGR2HSV);
    
    cv::Mat lowerRedMask, upperRedMask, redMask;
    cv::inRange(hsvMat, cv::Scalar(0, 150, 120), cv::Scalar(10, 255, 255), lowerRedMask);
    cv::inRange(hsvMat, cv::Scalar(160, 150, 120), cv::Scalar(179, 255, 255), upperRedMask);
    cv::addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, redMask);
    
    cv::GaussianBlur(redMask, redMask, cv::Size(9, 9), 2, 2);
    
    std::vector<cv::Vec3f> circles;
//    cv::HoughCircles(redMask, circles, cv::HOUGH_GRADIENT, 1, redMask.rows/8, 100, 30, 0, 0);
    cv::HoughCircles(redMask, circles, cv::HOUGH_GRADIENT, 1, redMask.rows/8, 100, 20, 0, 0);

    
    CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
    float *depthBase = (float *)CVPixelBufferGetBaseAddress(depthBuffer);
    int depthBytesPerRow = CVPixelBufferGetBytesPerRow(depthBuffer) / sizeof(float);
    size_t depthWidth = CVPixelBufferGetWidth(depthBuffer);
    size_t depthHeight = CVPixelBufferGetHeight(depthBuffer);
    cv::Mat depthMat(depthHeight, depthWidth, CV_32FC1, depthBase, depthBytesPerRow * sizeof(float));
    CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
    
    NSMutableArray<NSDictionary *> *result = [NSMutableArray array];
    int maxCircles = (int)std::min(3, (int)circles.size());
    if (maxCircles < 3) {
        printf("Less than 3 circles detected.\n");
        for (int i = 0; i < maxCircles; i++) {
            
            NSDictionary *dict = @{
                @"pixel_x": @(0.0f),
                @"pixel_y": @(0.0f),
                @"radius": @(0.0f),
                @"depth": @(0.0f),
                @"triangle_x": @(0.0f),
                @"triangle_y": @(0.0f),
                @"triangle_z": @(0.0f),
                @"lidar_x": @(0.0f),
                @"lidar_y": @(0.0f),
                @"lidar_z": @(0.0f),
                @"camera_a": @(0.0f),
                @"camera_b": @(0.0f),
                @"camera_c": @(0.0f),
                @"camera_d": @(0.0f),
                @"camera_x": @(0.0f),
                @"camera_y": @(0.0f),
                @"camera_z": @(0.0f)
            };
            [result addObject:dict];
            printf("Less than 3 circles");
        }
        return result;
    }
    
    // 1. 원들의 x,y 좌표와 depth 저장
    struct CircleInfo {
        float x, y, radius, depth;
    };
    std::vector<CircleInfo> points2DDepth;
    for (int i = 0; i < maxCircles; i++) {
        float x = circles[i][0];
        float y = circles[i][1];
        float radius = circles[i][2];
        
        int dx = (int)round(x * ((float)depthWidth / rgbWidth));
        int dy = (int)round(y * ((float)depthHeight / rgbHeight));
        
        float depth = 0.0f;
        if (dx >= 0 && dx < depthWidth && dy >= 0 && dy < depthHeight) {
            depth = depthMat.at<float>(dy, dx);
        }
        
        points2DDepth.push_back({x, y, radius, depth * 100.0f});
    }
    
    int originIndex = 0;
    float minY = points2DDepth[0].y;
    for (int i = 1; i < maxCircles; i++) {
        if (points2DDepth[i].y < minY) {
            minY = points2DDepth[i].y;
            originIndex = i;
        }
    }
    
    std::vector<int> otherIndices;
    for (int i = 0; i < maxCircles; i++) {
        if (i != originIndex) otherIndices.push_back(i);
    }
    
    // 원점과 x값 차이가 가장 적은 점을 point_x
    int pointXIndex = otherIndices[0];
    int pointYIndex = otherIndices[1];
    float diffX0 = std::abs(points2DDepth[originIndex].x - points2DDepth[otherIndices[0]].x);
    float diffX1 = std::abs(points2DDepth[originIndex].x - points2DDepth[otherIndices[1]].x);
    if (diffX1 < diffX0) {
        pointXIndex = otherIndices[1];
        pointYIndex = otherIndices[0];
    }
    
    // 3. 삼각형 좌표계 (cm 단위)
    // origin: (0,0,0)
    // point_x: (4,0,0)
    // point_y: (2, 2*sqrt(3), 0)
    struct TriCoord { float x, y, z; };
    std::vector<TriCoord> triangleCoords;
    triangleCoords.push_back({4.0f, 0.0f, 0.0f});
    triangleCoords.push_back({0.0f, 0.0f,  0.0f});
    triangleCoords.push_back({2.0f, 2.0f * std::sqrt(3.0f), 0.0f});
    
    std::vector<int> order = {originIndex, pointXIndex, pointYIndex};
    std::vector<cv::Point3f> measuredPts;
    
    cv::Matx33f K(
        intrinsicsArray[0], intrinsicsArray[3], intrinsicsArray[6],
        intrinsicsArray[1], intrinsicsArray[4], intrinsicsArray[7],
        intrinsicsArray[2], intrinsicsArray[5], intrinsicsArray[8]
    );
    float fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);
    
    cv::Matx44f cameraPose_world(
        cameraMatrix[0], cameraMatrix[4], cameraMatrix[8],  cameraMatrix[12],
        cameraMatrix[1], cameraMatrix[5], cameraMatrix[9],  cameraMatrix[13],
        cameraMatrix[2], cameraMatrix[6], cameraMatrix[10], cameraMatrix[14],
        cameraMatrix[3], cameraMatrix[7], cameraMatrix[11], cameraMatrix[15]
    );
    // (2, -15, 0)의 오프셋 (단위: 카메라 좌표계, cm라면 변환 필요)

    cv::Mat lidarPos_world = (cv::Mat_<double>(3,1) <<
            lidarWorldArray[0], lidarWorldArray[1], lidarWorldArray[2]);
    for (int i = 0; i < 3; i++) {
        int idx = order[i];
        float u = points2DDepth[idx].x;
        float v = points2DDepth[idx].y;
//        float Z = points2DDepth[idx].depth;
        
        float denominator = sqrt( pow((u - cx)/fx, 2) + pow((v - cy)/fy, 2) + 1 );
        float Z = points2DDepth[idx].depth / denominator;

        float X = - (v - cy) / fy * Z;        // X: 부호 그대로
        float Y = (u - cx) / fx * Z;
        measuredPts.push_back(cv::Point3f(X, Y, Z));
    }
    std::vector<cv::Point3f> triPts = {
        {4.0f, 0.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {2.0f, 2.0f * std::sqrt(3.0f), 0.0f}
    };
    cv::Mat offset_cam = (cv::Mat_<float>(4,1) << 0.02f, -0.15f, 0.0f, 1.0f);
    // 카메라 pose를 float로 사용 중이므로, Matx44f → Mat 변환
    cv::Mat offset_world = cv::Mat(cameraPose_world) * offset_cam; // 4x1
    
    std::vector<cv::Point3f> measuredPts_world;
    for (int i = 0; i < 3; ++i) {
        // pt_cam을 float로 생성
        cv::Mat pt_cam = (cv::Mat_<float>(4,1) << measuredPts[i].x, measuredPts[i].y, measuredPts[i].z, 1.0f);
        // cameraPose_world가 cv::Matx44f라면 Mat으로 변환해서 곱셈
        cv::Mat pt_world = cv::Mat(cameraPose_world) * pt_cam; // 4x1
        measuredPts_world.push_back(cv::Point3f(
            pt_world.at<float>(0,0),
            pt_world.at<float>(1,0),
            pt_world.at<float>(2,0)
        ));
    }

    cv::Mat R, t;
    rigid_transform_3D(triPts, measuredPts_world, R, t);
    
    cv::Mat R_inv = R.t();
    cv::Mat t_inv = -R_inv * t;
    cv::Mat cameraPos_world = (cv::Mat_<double>(3,1) <<
        cameraPose_world(0,3), cameraPose_world(1,3), cameraPose_world(2,3));
    cv::Mat cameraPos_tri = R_inv * (cameraPos_world - t);
    cv::Mat lidarPos_tri = R_inv * (lidarPos_world - t);
    cv::Mat offsetPos_world =(cv::Mat_<double>(3,1) <<
      offset_world.at<double>(0), offset_world.at<double>(1), offset_world.at<double>(2));
    cv::Mat offset_tri = R_inv * (offsetPos_world - t);

//    horn_rigid_transform_3D(triPts,measuredPts,R,t);
//    horn_rigid_transform_3D(triPts,measuredPts,R,t);
    cv::Mat R_cam_world = (cv::Mat_<double>(3,3) <<
        cameraPose_world(0,0), cameraPose_world(0,1), cameraPose_world(0,2),
        cameraPose_world(1,0), cameraPose_world(1,1), cameraPose_world(1,2),
        cameraPose_world(2,0), cameraPose_world(2,1), cameraPose_world(2,2)
    );
    cv::Mat R_cam_tri = R_inv * R_cam_world; // 3x3
    cv::Mat R_tri_cam = R_cam_tri.t();
    cv::Quatd quat = cv::Quatd::createFromRotMat(R_tri_cam);
    cv::Mat normal_tri = R_cam_tri.col(2); // 3x1
    
    double A = normal_tri.at<double>(0);
    double B = normal_tri.at<double>(1);
    double C = normal_tri.at<double>(2);
    double x0 = cameraPos_tri.at<double>(0);
    double y0 = cameraPos_tri.at<double>(1);
    double z0 = cameraPos_tri.at<double>(2);
    double D = -(A * x0 + B * y0 + C * z0);
    
    for (int i = 0; i < 3; i++) {
        int idx = order[i];
        NSDictionary *dict = @{
            @"pixel_x": @(points2DDepth[idx].x),
            @"pixel_y": @(points2DDepth[idx].y),
            @"radius": @(points2DDepth[idx].radius),
            @"depth": @(points2DDepth[idx].depth),
            @"triangle_x": @(triangleCoords[i].x),
            @"triangle_y": @(triangleCoords[i].y),
            @"triangle_z": @(triangleCoords[i].z),
            @"lidar_x": @(t.at<double>(0)),
            @"lidar_y": @(t.at<double>(1)),
            @"lidar_z": @(t.at<double>(2)),
            @"camera_a": @(A),
            @"camera_b": @(B),
            @"camera_c": @(C),
            @"camera_d": @(D),
            @"camera_x": @(x0),
            @"camera_y": @(y0),
            @"camera_z": @(z0),
            @"quat_w": @(quat.w),
            @"quat_x": @(quat.x),
            @"quat_y": @(quat.y),
            @"quat_z": @(quat.z)
        };
        [result addObject:dict];
        printf("Point %d: pixel (%.2f, %.2f), radius: %f, depth %.3f m, triangle (%.2f, %.2f, %.3f) cm\n",
               i+1,
               points2DDepth[idx].x,
               points2DDepth[idx].y,
               points2DDepth[idx].radius,
               points2DDepth[idx].depth,
               triangleCoords[i].x,
               triangleCoords[i].y,
               triangleCoords[i].z);
    }
    
    return result;
}
+ (NSArray<NSDictionary *> *)detectRectangleRedCirclesInPixelBuffer:
(CVPixelBufferRef)pixelBuffer
                                               depthBuffer:
(CVPixelBufferRef)depthBuffer
                                           intrinsicsArray:
(const float *)intrinsicsArray
                                              cameraMatrix:
(const float *)cameraMatrix
                                           lidarWorldArray:
(const float *)lidarWorldArray
{
    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
    
    size_t rgbWidth = CVPixelBufferGetWidth(pixelBuffer);
    size_t rgbHeight = CVPixelBufferGetHeight(pixelBuffer);
    
    printf("width: %zu", rgbWidth);
    printf("height: %zu", rgbHeight);
    
    uint8_t *yPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
    uint8_t *uvPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1);
    
    size_t yPitch = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
    size_t uvPitch = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
    
    cv::Mat yMat((int)rgbHeight, (int)rgbWidth, CV_8UC1, yPlane, yPitch);
    cv::Mat uvMat((int)rgbHeight/2, (int)rgbWidth/2, CV_8UC2, uvPlane, uvPitch);
    
    cv::Mat yuvMat((int)(rgbHeight * 3 / 2), (int)rgbWidth, CV_8UC1);
    memcpy(yuvMat.data, yMat.data, rgbWidth * rgbHeight);
    memcpy(yuvMat.data + rgbWidth * rgbHeight, uvMat.data, rgbWidth * rgbHeight / 2);
    
    cv::Mat bgrMat;
    cv::cvtColor(yuvMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
    
    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
    
    cv::Mat hsvMat;
    cv::cvtColor(bgrMat, hsvMat, cv::COLOR_BGR2HSV);
    
    cv::Mat lowerRedMask, upperRedMask, redMask;
    cv::inRange(hsvMat, cv::Scalar(0, 150, 120), cv::Scalar(10, 255, 255), lowerRedMask);
    cv::inRange(hsvMat, cv::Scalar(160, 150, 120), cv::Scalar(179, 255, 255), upperRedMask);
    cv::addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, redMask);
    
    cv::GaussianBlur(redMask, redMask, cv::Size(9, 9), 2, 2);
    
    std::vector<cv::Vec3f> circles;
//    cv::HoughCircles(redMask, circles, cv::HOUGH_GRADIENT, 1, redMask.rows/8, 100, 30, 0, 0);
    cv::HoughCircles(redMask, circles, cv::HOUGH_GRADIENT, 1, redMask.rows/8, 100, 20, 0, 0);

    
    CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
    float *depthBase = (float *)CVPixelBufferGetBaseAddress(depthBuffer);
    int depthBytesPerRow = CVPixelBufferGetBytesPerRow(depthBuffer) / sizeof(float);
    size_t depthWidth = CVPixelBufferGetWidth(depthBuffer);
    size_t depthHeight = CVPixelBufferGetHeight(depthBuffer);
    cv::Mat depthMat(depthHeight, depthWidth, CV_32FC1, depthBase, depthBytesPerRow * sizeof(float));
    CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
    
    NSMutableArray<NSDictionary *> *result = [NSMutableArray array];
    int maxCircles = (int)std::min(4, (int)circles.size());
    if (maxCircles < 4) {
        printf("Less than 4 circles detected.\n");
        for (int i = 0; i < maxCircles; i++) {
            
            NSDictionary *dict = @{
                @"pixel_x": @(0.0f),
                @"pixel_y": @(0.0f),
                @"radius": @(0.0f),
                @"depth": @(0.0f),
                @"triangle_x": @(0.0f),
                @"triangle_y": @(0.0f),
                @"triangle_z": @(0.0f),
                @"lidar_x": @(0.0f),
                @"lidar_y": @(0.0f),
                @"lidar_z": @(0.0f),
                @"camera_a": @(0.0f),
                @"camera_b": @(0.0f),
                @"camera_c": @(0.0f),
                @"camera_d": @(0.0f),
                @"camera_x": @(0.0f),
                @"camera_y": @(0.0f),
                @"camera_z": @(0.0f)
            };
            [result addObject:dict];
            printf("Less than 4 circles");
        }
        return result;
    }
    
    // 1. 원들의 x,y 좌표와 depth 저장
    struct CircleInfo {
        float x, y, radius, depth;
    };
    std::vector<CircleInfo> points2DDepth;
    for (int i = 0; i < maxCircles; i++) {
        float x = circles[i][0];
        float y = circles[i][1];
        float radius = circles[i][2];
        
        int dx = (int)round(x * ((float)depthWidth / rgbWidth));
        int dy = (int)round(y * ((float)depthHeight / rgbHeight));
        
        float depth = 0.0f;
        if (dx >= 0 && dx < depthWidth && dy >= 0 && dy < depthHeight) {
            depth = depthMat.at<float>(dy, dx);
        }
        
        points2DDepth.push_back({x, y, radius, depth * 100.0f});
    }
    
    std::vector<int> indices(4);
    for (int i = 0; i < 4; ++i) indices[i] = i;

    // 1. x+y 합이 가장 작은 점 (왼쪽 위)
    int firstIdx = 0;
    float minSum = points2DDepth[0].x + points2DDepth[0].y;
    for (int i = 1; i < 4; ++i) {
        float sum = points2DDepth[i].x + points2DDepth[i].y;
        if (sum < minSum) {
            minSum = sum;
            firstIdx = i;
        }
    }

    // 2. 첫번째 픽셀과 x값 차이가 가장 적은 점
    int minXIdx = -1;
    float minXDiff = std::numeric_limits<float>::max();
    for (int i = 0; i < 4; ++i) {
        if (i == firstIdx) continue;
        float diff = std::abs(points2DDepth[i].x - points2DDepth[firstIdx].x);
        if (diff < minXDiff) {
            minXDiff = diff;
            minXIdx = i;
        }
    }

    // 3. x+y 합이 가장 큰 점 (오른쪽 아래)
    int maxSumIdx = 0;
    float maxSum = points2DDepth[0].x + points2DDepth[0].y;
    for (int i = 1; i < 4; ++i) {
        float sum = points2DDepth[i].x + points2DDepth[i].y;
        if (sum > maxSum) {
            maxSum = sum;
            maxSumIdx = i;
        }
    }

    // 4. 첫번째 픽셀과 y값 차이가 가장 적은 점
    int minYIdx = -1;
    float minYDiff = std::numeric_limits<float>::max();
    for (int i = 0; i < 4; ++i) {
        if (i == firstIdx) continue;
        float diff = std::abs(points2DDepth[i].y - points2DDepth[firstIdx].y);
        if (diff < minYDiff) {
            minYDiff = diff;
            minYIdx = i;
        }
    }

    // 중복 제거 (minXIdx, maxSumIdx, minYIdx가 겹칠 수 있음)
    std::vector<int> order;
    order.push_back(firstIdx);
    if (std::find(order.begin(), order.end(), minXIdx) == order.end()) order.push_back(minXIdx);
    if (std::find(order.begin(), order.end(), maxSumIdx) == order.end()) order.push_back(maxSumIdx);
    if (std::find(order.begin(), order.end(), minYIdx) == order.end()) order.push_back(minYIdx);

    // 만약 중복이 있어 4개가 안 되면, 남은 인덱스를 추가
    for (int i = 0; i < 4; ++i) {
        if (std::find(order.begin(), order.end(), i) == order.end()) order.push_back(i);
    }

    // order: [첫 픽셀, x차 최소, x+y합 최대, y차 최소] 순서의 인덱스
    
    // 3. 삼각형 좌표계 (cm 단위)
    // origin: (0,0,0)
    // point_x: (4,0,0)
    // point_y: (2, 2*sqrt(3), 0)
    struct TriCoord { float x, y, z; };
    std::vector<TriCoord> triangleCoords;
    triangleCoords.push_back({0.0f, 4.0f, 0.0f});
    triangleCoords.push_back({0.0f, 0.0f,  0.0f});
    triangleCoords.push_back({4.0f, 0.0f,  0.0f});
    triangleCoords.push_back({4.0f, 4.0f,  0.0f});
    
    std::vector<cv::Point3f> measuredPts;
    
    cv::Matx33f K(
        intrinsicsArray[0], intrinsicsArray[3], intrinsicsArray[6],
        intrinsicsArray[1], intrinsicsArray[4], intrinsicsArray[7],
        intrinsicsArray[2], intrinsicsArray[5], intrinsicsArray[8]
    );
    float fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);
    
    cv::Matx44f cameraPose_world(
        cameraMatrix[0], cameraMatrix[4], cameraMatrix[8],  cameraMatrix[12],
        cameraMatrix[1], cameraMatrix[5], cameraMatrix[9],  cameraMatrix[13],
        cameraMatrix[2], cameraMatrix[6], cameraMatrix[10], cameraMatrix[14],
        cameraMatrix[3], cameraMatrix[7], cameraMatrix[11], cameraMatrix[15]
    );
    // (2, -15, 0)의 오프셋 (단위: 카메라 좌표계, cm라면 변환 필요)

    cv::Mat lidarPos_world = (cv::Mat_<double>(3,1) <<
            lidarWorldArray[0], lidarWorldArray[1], lidarWorldArray[2]);
    for (int i = 0; i < maxCircles; i++) {
        int idx = order[i];
        float u = points2DDepth[idx].x;
        float v = points2DDepth[idx].y;
//        float Z = points2DDepth[idx].depth;
        
        float denominator = sqrt( pow((u - cx)/fx, 2) + pow((v - cy)/fy, 2) + 1 );
        float Z = points2DDepth[idx].depth / denominator;

        float X = - (v - cy) / fy * Z;        // X: 부호 그대로
        float Y = (u - cx) / fx * Z;
        measuredPts.push_back(cv::Point3f(X, Y, Z));
    }
    
    std::vector<cv::Point3f> triPts = {
        {0.0f, 4.0f, 0.0f},
        {0.0f, 0.0f, 0.0f},
        {4.0f, 0.0f, 0.0f},
        {4.0f, 4.0f, 0.0f}
    };
    cv::Mat offset_cam = (cv::Mat_<float>(4,1) << 0.02f, -0.15f, 0.0f, 1.0f);
    // 카메라 pose를 float로 사용 중이므로, Matx44f → Mat 변환
    cv::Mat offset_world = cv::Mat(cameraPose_world) * offset_cam; // 4x1
    
    std::vector<cv::Point3f> measuredPts_world;
    for (int i = 0; i < maxCircles; ++i) {
        // pt_cam을 float로 생성
        cv::Mat pt_cam = (cv::Mat_<float>(4,1) << measuredPts[i].x, measuredPts[i].y, measuredPts[i].z, 1.0f);
        // cameraPose_world가 cv::Matx44f라면 Mat으로 변환해서 곱셈
        cv::Mat pt_world = cv::Mat(cameraPose_world) * pt_cam; // 4x1
        measuredPts_world.push_back(cv::Point3f(
            pt_world.at<float>(0,0),
            pt_world.at<float>(1,0),
            pt_world.at<float>(2,0)
        ));
    }

    cv::Mat R, t;
    rigid_transform_3D(triPts, measuredPts_world, R, t);
    
    cv::Mat R_inv = R.t();
    cv::Mat t_inv = -R_inv * t;
    cv::Mat cameraPos_world = (cv::Mat_<double>(3,1) <<
        cameraPose_world(0,3), cameraPose_world(1,3), cameraPose_world(2,3));
    cv::Mat cameraPos_tri = R_inv * (cameraPos_world - t);
    cv::Mat lidarPos_tri = R_inv * (lidarPos_world - t);
    cv::Mat offsetPos_world =(cv::Mat_<double>(3,1) <<
      offset_world.at<double>(0), offset_world.at<double>(1), offset_world.at<double>(2));
    cv::Mat offset_tri = R_inv * (offsetPos_world - t);

//    horn_rigid_transform_3D(triPts,measuredPts,R,t);
//    horn_rigid_transform_3D(triPts,measuredPts,R,t);
    cv::Mat R_cam_world = (cv::Mat_<double>(3,3) <<
        cameraPose_world(0,0), cameraPose_world(0,1), cameraPose_world(0,2),
        cameraPose_world(1,0), cameraPose_world(1,1), cameraPose_world(1,2),
        cameraPose_world(2,0), cameraPose_world(2,1), cameraPose_world(2,2)
    );
    cv::Mat R_cam_tri = R_inv * R_cam_world; // 3x3
    cv::Mat normal_tri = R_cam_tri.col(2); // 3x1
    cv::Mat R_tri_cam = R_cam_tri.t();
    cv::Quatd quat = cv::Quatd::createFromRotMat(R_tri_cam);
    double A = normal_tri.at<double>(0);
    double B = normal_tri.at<double>(1);
    double C = normal_tri.at<double>(2);
    double x0 = cameraPos_tri.at<double>(0);
    double y0 = cameraPos_tri.at<double>(1);
    double z0 = cameraPos_tri.at<double>(2);
    double D = -(A * x0 + B * y0 + C * z0);
//    cv::Point3f camPos_tri(
//        cameraPose_tri(0,3),
//        cameraPose_tri(1,3),
//        cameraPose_tri(2,3)
//    );
//    cv::Vec3f normal_tri(
//        cameraPose_tri(0,2),
//        cameraPose_tri(1,2),
//        cameraPose_tri(2,2)
//    );
    

    for (int i = 0; i < maxCircles; i++) {
        int idx = order[i];
        NSDictionary *dict = @{
            @"pixel_x": @(points2DDepth[idx].x),
            @"pixel_y": @(points2DDepth[idx].y),
            @"radius": @(points2DDepth[idx].radius),
            @"depth": @(points2DDepth[idx].depth),
            @"triangle_x": @(triangleCoords[i].x),
            @"triangle_y": @(triangleCoords[i].y),
            @"triangle_z": @(triangleCoords[i].z),
            @"lidar_x": @(t.at<double>(0)),
            @"lidar_y": @(t.at<double>(1)),
            @"lidar_z": @(t.at<double>(2)),
            @"camera_a": @(A),
            @"camera_b": @(B),
            @"camera_c": @(C),
            @"camera_d": @(D),
            @"camera_x": @(x0),
            @"camera_y": @(y0),
            @"camera_z": @(z0),
            @"quat_w": @(quat.w),
            @"quat_x": @(quat.x),
            @"quat_y": @(quat.y),
            @"quat_z": @(quat.z)
        };
        [result addObject:dict];
        printf("Point %d: pixel (%.2f, %.2f), radius: %f, depth %.3f m, triangle (%.2f, %.2f, %.3f) cm\n",
               i+1,
               points2DDepth[idx].x,
               points2DDepth[idx].y,
               points2DDepth[idx].radius,
               points2DDepth[idx].depth,
               triangleCoords[i].x,
               triangleCoords[i].y,
               triangleCoords[i].z);
    }
    
    return result;
}

//+ (NSArray<NSDictionary *> *)detectHexRedCirclesInPixelBuffer:
//(CVPixelBufferRef)pixelBuffer
//                                               depthBuffer:
//(CVPixelBufferRef)depthBuffer
//                                           intrinsicsArray:
//(const float *)intrinsicsArray
//                                              cameraMatrix:
//(const float *)cameraMatrix
//                                           lidarWorldArray:
//(const float *)lidarWorldArray
//{
//    CVPixelBufferLockBaseAddress(pixelBuffer, 0);
//    
//    size_t rgbWidth = CVPixelBufferGetWidth(pixelBuffer);
//    size_t rgbHeight = CVPixelBufferGetHeight(pixelBuffer);
//    
//    printf("width: %zu", rgbWidth);
//    printf("height: %zu", rgbHeight);
//    
//    uint8_t *yPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 0);
//    uint8_t *uvPlane = (uint8_t *)CVPixelBufferGetBaseAddressOfPlane(pixelBuffer, 1);
//    
//    size_t yPitch = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 0);
//    size_t uvPitch = CVPixelBufferGetBytesPerRowOfPlane(pixelBuffer, 1);
//    
//    cv::Mat yMat((int)rgbHeight, (int)rgbWidth, CV_8UC1, yPlane, yPitch);
//    cv::Mat uvMat((int)rgbHeight/2, (int)rgbWidth/2, CV_8UC2, uvPlane, uvPitch);
//    
//    cv::Mat yuvMat((int)(rgbHeight * 3 / 2), (int)rgbWidth, CV_8UC1);
//    memcpy(yuvMat.data, yMat.data, rgbWidth * rgbHeight);
//    memcpy(yuvMat.data + rgbWidth * rgbHeight, uvMat.data, rgbWidth * rgbHeight / 2);
//    
//    cv::Mat bgrMat;
//    cv::cvtColor(yuvMat, bgrMat, cv::COLOR_YUV2BGR_NV12);
//    
//    CVPixelBufferUnlockBaseAddress(pixelBuffer, 0);
//    
//    cv::Mat hsvMat;
//    cv::cvtColor(bgrMat, hsvMat, cv::COLOR_BGR2HSV);
//    
//    cv::Mat lowerRedMask, upperRedMask, redMask;
//    cv::inRange(hsvMat, cv::Scalar(0, 150, 120), cv::Scalar(10, 255, 255), lowerRedMask);
//    cv::inRange(hsvMat, cv::Scalar(160, 150, 120), cv::Scalar(179, 255, 255), upperRedMask);
//    cv::addWeighted(lowerRedMask, 1.0, upperRedMask, 1.0, 0.0, redMask);
//    
//    cv::GaussianBlur(redMask, redMask, cv::Size(9, 9), 2, 2);
//    
//    std::vector<cv::Vec3f> circles;
////    cv::HoughCircles(redMask, circles, cv::HOUGH_GRADIENT, 1, redMask.rows/8, 100, 30, 0, 0);
//    cv::HoughCircles(redMask, circles, cv::HOUGH_GRADIENT, 1, redMask.rows/8, 100, 20, 0, 0);
//
//    
//    CVPixelBufferLockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
//    float *depthBase = (float *)CVPixelBufferGetBaseAddress(depthBuffer);
//    int depthBytesPerRow = CVPixelBufferGetBytesPerRow(depthBuffer) / sizeof(float);
//    size_t depthWidth = CVPixelBufferGetWidth(depthBuffer);
//    size_t depthHeight = CVPixelBufferGetHeight(depthBuffer);
//    cv::Mat depthMat(depthHeight, depthWidth, CV_32FC1, depthBase, depthBytesPerRow * sizeof(float));
//    CVPixelBufferUnlockBaseAddress(depthBuffer, kCVPixelBufferLock_ReadOnly);
//    
//    NSMutableArray<NSDictionary *> *result = [NSMutableArray array];
//    int maxCircles = (int)std::min(6, (int)circles.size());
//    if (maxCircles < 6) {
//        printf("Less than 6 circles detected.\n");
//        for (int i = 0; i < maxCircles; i++) {
//            
//            NSDictionary *dict = @{
//                @"pixel_x": @(0.0f),
//                @"pixel_y": @(0.0f),
//                @"radius": @(0.0f),
//                @"depth": @(0.0f),
//                @"triangle_x": @(0.0f),
//                @"triangle_y": @(0.0f),
//                @"triangle_z": @(0.0f),
//                @"lidar_x": @(0.0f),
//                @"lidar_y": @(0.0f),
//                @"lidar_z": @(0.0f),
//                @"camera_a": @(0.0f),
//                @"camera_b": @(0.0f),
//                @"camera_c": @(0.0f),
//                @"camera_d": @(0.0f),
//                @"camera_x": @(0.0f),
//                @"camera_y": @(0.0f),
//                @"camera_z": @(0.0f)
//            };
//            [result addObject:dict];
//            printf("Less than 6 circles");
//        }
//        return result;
//    }
//    
//    // 1. 원들의 x,y 좌표와 depth 저장
//    struct CircleInfo {
//        float x, y, radius, depth;
//    };
//    std::vector<CircleInfo> points2DDepth;
//    for (int i = 0; i < maxCircles; i++) {
//        float x = circles[i][0];
//        float y = circles[i][1];
//        float radius = circles[i][2];
//        
//        int dx = (int)round(x * ((float)depthWidth / rgbWidth));
//        int dy = (int)round(y * ((float)depthHeight / rgbHeight));
//        
//        float depth = 0.0f;
//        if (dx >= 0 && dx < depthWidth && dy >= 0 && dy < depthHeight) {
//            depth = depthMat.at<float>(dy, dx);
//        }
//        
//        points2DDepth.push_back({x, y, radius, depth * 100.0f});
//    }
//    
//    // points2DDepth: 6개의 cv::Point2f (또는 x, y 멤버가 있는 구조체)
//    std::vector<int> order;
//
//    // 1. 첫 번째 점: y값이 가장 작은 점
//    int firstIdx = 0;
//    float minY = points2DDepth[0].y;
//    for (int i = 1; i < 6; ++i) {
//        if (points2DDepth[i].y < minY) {
//            minY = points2DDepth[i].y;
//            firstIdx = i;
//        }
//    }
//    order.push_back(firstIdx);
//
//    // 2. 두 번째 점: 첫 번째 점과 y값 차이가 가장 적고, x값이 첫 번째 점보다 작은 점
//    int secondIdx = -1;
//    float minYDiff = std::numeric_limits<float>::max();
//    for (int i = 0; i < 6; ++i) {
//        if (i == firstIdx) continue;
//        float yDiff = std::abs(points2DDepth[i].y - points2DDepth[firstIdx].y);
//        if (points2DDepth[i].x < points2DDepth[firstIdx].x && yDiff < minYDiff) {
//            minYDiff = yDiff;
//            secondIdx = i;
//        }
//    }
//    // 만약 조건을 만족하는 점이 없으면, y값 차이가 가장 적은 점을 선택
//    if (secondIdx == -1) {
//        minYDiff = std::numeric_limits<float>::max();
//        for (int i = 0; i < 6; ++i) {
//            if (i == firstIdx) continue;
//            float yDiff = std::abs(points2DDepth[i].y - points2DDepth[firstIdx].y);
//            if (yDiff < minYDiff) {
//                minYDiff = yDiff;
//                secondIdx = i;
//            }
//        }
//    }
//    order.push_back(secondIdx);
//
//    // 3. 세 번째 점: 두 번째 점과 x값 차이가 가장 적은 점
//    int thirdIdx = -1;
//    float minXDiff = std::numeric_limits<float>::max();
//    for (int i = 0; i < 6; ++i) {
//        if (i == firstIdx || i == secondIdx) continue;
//        float xDiff = std::abs(points2DDepth[i].x - points2DDepth[secondIdx].x);
//        if (xDiff < minXDiff) {
//            minXDiff = xDiff;
//            thirdIdx = i;
//        }
//    }
//    order.push_back(thirdIdx);
//
//    // 4. 네 번째 점: 첫 번째 점과 x값 차이가 가장 적은 점
//    int fourthIdx = -1;
//    minXDiff = std::numeric_limits<float>::max();
//    for (int i = 0; i < 6; ++i) {
//        if (i == firstIdx || i == secondIdx || i == thirdIdx) continue;
//        float xDiff = std::abs(points2DDepth[i].x - points2DDepth[firstIdx].x);
//        if (xDiff < minXDiff) {
//            minXDiff = xDiff;
//            fourthIdx = i;
//        }
//    }
//    order.push_back(fourthIdx);
//
//    // 5. 다섯 번째 점: 세 번째 점과 y값 차이가 가장 적은 점
//    int fifthIdx = -1;
//    minYDiff = std::numeric_limits<float>::max();
//    for (int i = 0; i < 6; ++i) {
//        if (i == firstIdx || i == secondIdx || i == thirdIdx || i == fourthIdx) continue;
//        float yDiff = std::abs(points2DDepth[i].y - points2DDepth[thirdIdx].y);
//        if (yDiff < minYDiff) {
//            minYDiff = yDiff;
//            fifthIdx = i;
//        }
//    }
//    order.push_back(fifthIdx);
//
//    // 6. 여섯 번째 점: 다섯 번째 점과 x값 차이가 가장 적은 점
//    int sixthIdx = -1;
//    minXDiff = std::numeric_limits<float>::max();
//    for (int i = 0; i < 6; ++i) {
//        if (i == firstIdx || i == secondIdx || i == thirdIdx || i == fourthIdx || i == fifthIdx) continue;
//        float xDiff = std::abs(points2DDepth[i].x - points2DDepth[fifthIdx].x);
//        if (xDiff < minXDiff) {
//            minXDiff = xDiff;
//            sixthIdx = i;
//        }
//    }
//    order.push_back(sixthIdx);
//    // 3. 삼각형 좌표계 (cm 단위)
//    // origin: (0,0,0)
//    // point_x: (4,0,0)
//    // point_y: (2, 2*sqrt(3), 0)
//    struct TriCoord { float x, y, z; };
//    std::vector<TriCoord> triangleCoords;
//    triangleCoords.push_back({4.5f, 1.5f*std::sqrt(3.0f), 0.0f});
//    triangleCoords.push_back({3.0f, 3.0f*std::sqrt(3.0f),  0.0f});
//    triangleCoords.push_back({0.0f, 3.0f*std::sqrt(3.0f),  0.0f});
//    triangleCoords.push_back({-1.5f, 1.5f*std::sqrt(3.0f),  0.0f});
//    triangleCoords.push_back({0.0f, 0.0f,  0.0f});
//    triangleCoords.push_back({3.0f, 0.0f,  0.0f});
//    
//    std::vector<cv::Point3f> measuredPts;
//    
//    cv::Matx33f K(
//        intrinsicsArray[0], intrinsicsArray[3], intrinsicsArray[6],
//        intrinsicsArray[1], intrinsicsArray[4], intrinsicsArray[7],
//        intrinsicsArray[2], intrinsicsArray[5], intrinsicsArray[8]
//    );
//    float fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);
//    
//    cv::Matx44f cameraPose_world(
//        cameraMatrix[0], cameraMatrix[4], cameraMatrix[8],  cameraMatrix[12],
//        cameraMatrix[1], cameraMatrix[5], cameraMatrix[9],  cameraMatrix[13],
//        cameraMatrix[2], cameraMatrix[6], cameraMatrix[10], cameraMatrix[14],
//        cameraMatrix[3], cameraMatrix[7], cameraMatrix[11], cameraMatrix[15]
//    );
//    // (2, -15, 0)의 오프셋 (단위: 카메라 좌표계, cm라면 변환 필요)
//
//    cv::Mat lidarPos_world = (cv::Mat_<double>(3,1) <<
//            lidarWorldArray[0], lidarWorldArray[1], lidarWorldArray[2]);
//    for (int i = 0; i < maxCircles; i++) {
//        int idx = order[i];
//        float u = points2DDepth[idx].x;
//        float v = points2DDepth[idx].y;
////        float Z = points2DDepth[idx].depth;
//        
//        float denominator = sqrt( pow((u - cx)/fx, 2) + pow((v - cy)/fy, 2) + 1 );
//        float Z = points2DDepth[idx].depth / denominator;
//
//        float X = - (v - cy) / fy * Z;        // X: 부호 그대로
//        float Y = (u - cx) / fx * Z;
//        measuredPts.push_back(cv::Point3f(X, Y, Z));
//    }
//
//    std::vector<cv::Point3f> triPts = {
//        {4.5f, 1.5f*std::sqrt(3.0f), 0.0f},
//        {3.0f, 3.0f*std::sqrt(3.0f),  0.0f},
//        {0.0f, 3.0f*std::sqrt(3.0f),  0.0f},
//        {-1.5f, 1.5f*std::sqrt(3.0f),  0.0f},
//        {0.0f, 0.0f,  0.0f},
//        {3.0f, 0.0f,  0.0f}
//    };
//    cv::Mat offset_cam = (cv::Mat_<float>(4,1) << 0.02f, -0.15f, 0.0f, 1.0f);
//    // 카메라 pose를 float로 사용 중이므로, Matx44f → Mat 변환
//    cv::Mat offset_world = cv::Mat(cameraPose_world) * offset_cam; // 4x1
//    
//    std::vector<cv::Point3f> measuredPts_world;
//    for (int i = 0; i < maxCircles; ++i) {
//        // pt_cam을 float로 생성
//        cv::Mat pt_cam = (cv::Mat_<float>(4,1) << measuredPts[i].x, measuredPts[i].y, measuredPts[i].z, 1.0f);
//        // cameraPose_world가 cv::Matx44f라면 Mat으로 변환해서 곱셈
//        cv::Mat pt_world = cv::Mat(cameraPose_world) * pt_cam; // 4x1
//        measuredPts_world.push_back(cv::Point3f(
//            pt_world.at<float>(0,0),
//            pt_world.at<float>(1,0),
//            pt_world.at<float>(2,0)
//        ));
//    }
//
//    cv::Mat R, t;
//    rigid_transform_3D(triPts, measuredPts_world, R, t);
//    
//    cv::Mat R_inv = R.t();
//    cv::Mat t_inv = -R_inv * t;
//    cv::Mat cameraPos_world = (cv::Mat_<double>(3,1) <<
//        cameraPose_world(0,3), cameraPose_world(1,3), cameraPose_world(2,3));
//    cv::Mat cameraPos_tri = R_inv * (cameraPos_world - t);
//    cv::Mat lidarPos_tri = R_inv * (lidarPos_world - t);
//    cv::Mat offsetPos_world =(cv::Mat_<double>(3,1) <<
//      offset_world.at<double>(0), offset_world.at<double>(1), offset_world.at<double>(2));
//    cv::Mat offset_tri = R_inv * (offsetPos_world - t);
//
//    cv::Mat R_cam_world = (cv::Mat_<double>(3,3) <<
//        cameraPose_world(0,0), cameraPose_world(0,1), cameraPose_world(0,2),
//        cameraPose_world(1,0), cameraPose_world(1,1), cameraPose_world(1,2),
//        cameraPose_world(2,0), cameraPose_world(2,1), cameraPose_world(2,2)
//    );
//    cv::Mat R_cam_tri = R_inv * R_cam_world; // 3x3
//    cv::Mat normal_tri = R_cam_tri.col(2); // 3x1
//    cv::Mat R_tri_cam = R_cam_tri.t();
//    cv::Quatd quat = cv::Quatd::createFromRotMat(R_tri_cam);
//
//    double A = normal_tri.at<double>(0);
//    double B = normal_tri.at<double>(1);
//    double C = normal_tri.at<double>(2);
//    double x0 = cameraPos_tri.at<double>(0);
//    double y0 = cameraPos_tri.at<double>(1);
//    double z0 = cameraPos_tri.at<double>(2);
//    double D = -(A * x0 + B * y0 + C * z0);
//
//    for (int i = 0; i < maxCircles; i++) {
//        int idx = order[i];
//        NSDictionary *dict = @{
//            @"pixel_x": @(points2DDepth[idx].x),
//            @"pixel_y": @(points2DDepth[idx].y),
//            @"radius": @(points2DDepth[idx].radius),
//            @"depth": @(points2DDepth[idx].depth),
//            @"triangle_x": @(triangleCoords[i].x),
//            @"triangle_y": @(triangleCoords[i].y),
//            @"triangle_z": @(triangleCoords[i].z),
//            @"lidar_x": @(t.at<double>(0)),
//            @"lidar_y": @(t.at<double>(1)),
//            @"lidar_z": @(t.at<double>(2)),
//            @"camera_a": @(A),
//            @"camera_b": @(B),
//            @"camera_c": @(C),
//            @"camera_d": @(D),
//            @"camera_x": @(x0),
//            @"camera_y": @(y0),
//            @"camera_z": @(z0),
//            @"quat_w": @(quat.w),
//            @"quat_x": @(quat.x),
//            @"quat_y": @(quat.y),
//            @"quat_z": @(quat.z)
//        };
//        [result addObject:dict];
//        printf("Point %d: pixel (%.2f, %.2f), radius: %f, depth %.3f m, triangle (%.2f, %.2f, %.3f) cm\n",
//               i+1,
//               points2DDepth[idx].x,
//               points2DDepth[idx].y,
//               points2DDepth[idx].radius,
//               points2DDepth[idx].depth,
//               triangleCoords[i].x,
//               triangleCoords[i].y,
//               triangleCoords[i].z);
//    }
//    
//    return result;
//}



void rigid_transform_3D(const std::vector<cv::Point3f>& src, const std::vector<cv::Point3f>& dst, cv::Mat& R, cv::Mat& t) {
    CV_Assert(src.size() == dst.size());
    int N = (int)src.size();
    
    // 중심 계산
    cv::Point3f centroid_src(0,0,0), centroid_dst(0,0,0);
    for(int i=0; i<N; ++i) {
        centroid_src += src[i];
        centroid_dst += dst[i];
    }
    centroid_src *= (1.0/N);
    centroid_dst *= (1.0/N);
    
    // 공분산 행렬 H
    cv::Mat H = cv::Mat::zeros(3,3,CV_64F);
    for(int i=0; i<N; ++i) {
        cv::Mat p = (cv::Mat_<double>(3,1) << src[i].x - centroid_src.x, src[i].y - centroid_src.y, src[i].z - centroid_src.z);
        cv::Mat q = (cv::Mat_<double>(3,1) << dst[i].x - centroid_dst.x, dst[i].y - centroid_dst.y, dst[i].z - centroid_dst.z);
        H += p * q.t();
    }
    
    // SVD
    cv::SVD svd(H);
    R = svd.vt.t() * svd.u.t();
    
    // Reflection 보정
    if (cv::determinant(R) < 0) {
        cv::Mat B = cv::Mat::eye(3,3,CV_64F);
        B.at<double>(2,2) = -1;
        R = svd.vt.t() * B * svd.u.t();
    }
    
    // 이동 벡터
    cv::Mat centroid_src_mat = (cv::Mat_<double>(3,1) << centroid_src.x, centroid_src.y, centroid_src.z);
    cv::Mat centroid_dst_mat = (cv::Mat_<double>(3,1) << centroid_dst.x, centroid_dst.y, centroid_dst.z);
    t = centroid_dst_mat - R * centroid_src_mat;
}

void horn_rigid_transform_3D(const std::vector<cv::Point3f>& src,
                             const std::vector<cv::Point3f>& dst,
                             cv::Mat& R, cv::Mat& t) {
    CV_Assert(src.size() == dst.size());
    int N = (int)src.size();

    // 1. 중심 계산
    cv::Point3f centroid_src(0,0,0), centroid_dst(0,0,0);
    for(int i=0; i<N; ++i) {
        centroid_src += src[i];
        centroid_dst += dst[i];
    }
    centroid_src *= (1.0/N);
    centroid_dst *= (1.0/N);

    // 2. 중심화
    std::vector<cv::Point3f> src_centered(N), dst_centered(N);
    for(int i=0; i<N; ++i) {
        src_centered[i] = src[i] - centroid_src;
        dst_centered[i] = dst[i] - centroid_dst;
    }

    // 3. 공분산 행렬 H
    cv::Mat H = cv::Mat::zeros(3,3,CV_64F);
    for(int i=0; i<N; ++i) {
        cv::Mat p = (cv::Mat_<double>(3,1) << src_centered[i].x, src_centered[i].y, src_centered[i].z);
        cv::Mat q = (cv::Mat_<double>(1,3) << dst_centered[i].x, dst_centered[i].y, dst_centered[i].z);
        H += p * q;
    }

    // 4. 4x4 대칭 행렬 Q 생성 (Horn's Method)
    double traceH = H.at<double>(0,0) + H.at<double>(1,1) + H.at<double>(2,2);
    cv::Mat delta = (cv::Mat_<double>(3,1) <<
        H.at<double>(1,2) - H.at<double>(2,1),
        H.at<double>(2,0) - H.at<double>(0,2),
        H.at<double>(0,1) - H.at<double>(1,0));

    cv::Mat Q = cv::Mat::zeros(4,4,CV_64F);
    Q.at<double>(0,0) = traceH;
    Q.at<double>(0,1) = delta.at<double>(0,0);
    Q.at<double>(0,2) = delta.at<double>(1,0);
    Q.at<double>(0,3) = delta.at<double>(2,0);
    Q.at<double>(1,0) = delta.at<double>(0,0);
    Q.at<double>(2,0) = delta.at<double>(1,0);
    Q.at<double>(3,0) = delta.at<double>(2,0);

    for(int i=1; i<4; ++i) {
        for(int j=1; j<4; ++j) {
            Q.at<double>(i,j) = H.at<double>(i-1,j-1) + H.at<double>(j-1,i-1);
        }
    }
    for(int i=1; i<4; ++i) {
        Q.at<double>(i,i) -= traceH;
    }

    // 5. Q의 고유값/고유벡터 계산 (최대 고유값의 고유벡터가 쿼터니언)
    cv::Mat eigenvalues, eigenvectors;
    cv::eigen(Q, eigenvalues, eigenvectors);

    // 6. 쿼터니언 → 회전행렬
    cv::Mat q = eigenvectors.row(0).t(); // 첫 번째 행(최대 고유값)
    double w = q.at<double>(0,0);
    double x = q.at<double>(1,0);
    double y = q.at<double>(2,0);
    double z = q.at<double>(3,0);

    R = (cv::Mat_<double>(3,3) <<
        1 - 2*(y*y + z*z),     2*(x*y - z*w),       2*(x*z + y*w),
        2*(x*y + z*w),         1 - 2*(x*x + z*z),   2*(y*z - x*w),
        2*(x*z - y*w),         2*(y*z + x*w),       1 - 2*(x*x + y*y)
    );

    // 7. 이동 벡터
    cv::Mat centroid_src_mat = (cv::Mat_<double>(3,1) << centroid_src.x, centroid_src.y, centroid_src.z);
    cv::Mat centroid_dst_mat = (cv::Mat_<double>(3,1) << centroid_dst.x, centroid_dst.y, centroid_dst.z);
    t = centroid_dst_mat - R * centroid_src_mat;
}
@end
