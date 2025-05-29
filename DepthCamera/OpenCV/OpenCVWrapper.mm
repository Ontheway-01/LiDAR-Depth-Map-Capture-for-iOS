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

@implementation OpenCVWrapper

+ (NSArray<NSDictionary *> *)detectRedCirclesInPixelBuffer:
(CVPixelBufferRef)pixelBuffer
                                               depthBuffer:
(CVPixelBufferRef)depthBuffer
                                           intrinsicsArray:
(const float *)intrinsicsArray
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
                @"lidar_z": @(0.0f)
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
    triangleCoords.push_back({0.0f, 0.0f, 0.0f});
    triangleCoords.push_back({4.0f, 0.0f,  0.0f});
    triangleCoords.push_back({2.0f, 2.0f * std::sqrt(3.0f), 0.0f});
    
    std::vector<int> order = {originIndex, pointXIndex, pointYIndex};
    std::vector<cv::Point3f> measuredPts;
    
    cv::Matx33f K(
        intrinsicsArray[0], intrinsicsArray[3], intrinsicsArray[6],
        intrinsicsArray[1], intrinsicsArray[4], intrinsicsArray[7],
        intrinsicsArray[2], intrinsicsArray[5], intrinsicsArray[8]
    );
    float fx = K(0,0), fy = K(1,1), cx = K(0,2), cy = K(1,2);
    
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
        {0.0f, 0.0f, 0.0f},
        {4.0f, 0.0f, 0.0f},
        {2.0f, 2.0f * std::sqrt(3.0f), 0.0f}
    };
    cv::Mat R, t;
    rigid_transform_3D(triPts, measuredPts, R, t);
//    horn_rigid_transform_3D(triPts,measuredPts,R,t);
//    horn_rigid_transform_3D(triPts,measuredPts,R,t);
    
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
            @"lidar_x": @(t.at<double>(0,0)),
            @"lidar_y": @(t.at<double>(1,0)),
            @"lidar_z": @(t.at<double>(2,0))        };
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
