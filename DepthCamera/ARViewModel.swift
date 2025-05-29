//
//  ARViewModel.swift
//  DepthCamera
//
//  Created by iori on 2024/11/27.
//

import ARKit
import SwiftUI

struct CircleData {
    let pixelX: Double
    let pixelY: Double
    let radius: Double
    let depth: Double
    let triangleX: Double
    let triangleY: Double
    let triangleZ: Double
}

struct LidarData {
    let lidarX: Double
    let lidarY: Double
    let lidarZ: Double
}

class ARViewModel: NSObject, ARSessionDelegate, ObservableObject {
    let lidarSender = LidarSender(host: "192.168.1.38", port: 5005)
    private var latestDepthMap: CVPixelBuffer?
    private var latestImage: CVPixelBuffer?
    private var latestCameraIntrinsics: simd_float3x3?
    @Published var lastCapture: UIImage? = nil {
        didSet {
            print("lastCapture was set.")
        }
    }
    @Published var detectedCircles: [CircleData] = []
    private var lidarPosition: LidarData = LidarData(lidarX: 0.0, lidarY: 0.0, lidarZ: 0.0)
    @Published var pixelBufferWidth: Int?
    @Published var pixelBufferHeight: Int?
    
    private var lastProcessedTimestamp: TimeInterval = 0
    // 원하는 샘플링 간격(초): 0.05는 20fps, 0.1은 10fps
    private let minInterval: TimeInterval = 0.05
    
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        let currentTimestamp = frame.timestamp
        if currentTimestamp - lastProcessedTimestamp < minInterval {
            return // 샘플링 간격보다 빠르면 처리하지 않음
        }
        lastProcessedTimestamp = currentTimestamp
        
        let pixelBuffer = frame.capturedImage
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        latestCameraIntrinsics = frame.camera.intrinsics
        
        print("RGB: \(latestCameraIntrinsics)")
        
        latestDepthMap = frame.sceneDepth?.depthMap
        guard let depthMap = latestDepthMap else {return}
        latestImage = frame.capturedImage
        if let sceneDepth = frame.sceneDepth {
            let formatType = CVPixelBufferGetPixelFormatType(sceneDepth.depthMap)
            print("Pixel Format:", formatType == kCVPixelFormatType_DepthFloat32 ? "Float32 (OK)" : "Other")

            if sceneDepth.confidenceMap != nil {
                print("Likely LiDAR-based depth")
            } else {
                print("Could be neural depth (non-LiDAR)")
            }
        }
        guard let intrinsics = latestCameraIntrinsics else {return}
        let intrinsicsArray: [Float] = [
            intrinsics.columns.0.x, intrinsics.columns.0.y, intrinsics.columns.0.z,
            intrinsics.columns.1.x, intrinsics.columns.1.y, intrinsics.columns.1.z,
            intrinsics.columns.2.x, intrinsics.columns.2.y, intrinsics.columns.2.z
        ]
        DispatchQueue.main.async { [weak self] in
            self?.pixelBufferWidth = width
            self?.pixelBufferHeight = height
        }


        DispatchQueue.global(qos: .userInitiated).async {
            let results = OpenCVWrapper.detectRedCircles(in: pixelBuffer, depthBuffer: depthMap, intrinsicsArray: intrinsicsArray) as? [[String: NSNumber]]

            var circles: [CircleData] = []
            var lidar: LidarData = LidarData(lidarX: 0.0, lidarY: 0.0, lidarZ: 0.0)

            results?.forEach { dict in
                if let px = dict["pixel_x"]?.doubleValue,
                   let py = dict["pixel_y"]?.doubleValue,
                   let radius = dict["radius"]?.doubleValue,
                   let depth = dict["depth"]?.doubleValue,
                   let tx = dict["triangle_x"]?.doubleValue,
                   let ty = dict["triangle_y"]?.doubleValue,
                   let tz = dict["triangle_z"]?.doubleValue,
                   let lx = dict["lidar_x"]?.doubleValue,
                   let ly = dict["lidar_y"]?.doubleValue,
                   let lz = dict["lidar_z"]?.doubleValue{
                    let circle = CircleData(pixelX: px, pixelY: py, radius: radius, depth: depth, triangleX: tx, triangleY: ty, triangleZ: tz)
                    lidar = LidarData(lidarX: lx, lidarY: ly, lidarZ: lz)
                    circles.append(circle)
                }
            }

            DispatchQueue.main.async {
                self.detectedCircles = circles
                self.lidarPosition = lidar
//                self.lidarSender.sendLidarPosition(x: self.lidarPosition.lidarX, y: self.lidarPosition.lidarY, z: self.lidarPosition.lidarZ)
//                print("LiDAR Coor: (\(self.lidarPosition.lidarX), \(self.lidarPosition.lidarY), \(self.lidarPosition.lidarZ))")                // Print mapping
//                for (index, circle) in circles.enumerated() {
//                    print("Circle \(index + 1): Pixel (\(circle.pixelX), \(circle.pixelY)), Radius: \(circle.radius), Depth: \(circle.depth)m, Triangle Coord: (\(circle.triangleX), \(circle.triangleY), \(circle.triangleZ)) cm")
//                }
            }
        }
    }
    func sendCurrentLidarPositionIfAvailable() {
        // 최신 데이터가 모두 있는지 확인
        guard let pixelBuffer = latestImage,
              let depthMap = latestDepthMap,
              let intrinsics = latestCameraIntrinsics else {
            print("필요한 데이터가 없습니다.")
            return
        }
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)
        let intrinsicsArray: [Float] = [
            intrinsics.columns.0.x, intrinsics.columns.0.y, intrinsics.columns.0.z,
            intrinsics.columns.1.x, intrinsics.columns.1.y, intrinsics.columns.1.z,
            intrinsics.columns.2.x, intrinsics.columns.2.y, intrinsics.columns.2.z
        ]
        // LiDAR 계산 (OpenCVWrapper 등 활용)
        DispatchQueue.global(qos: .userInitiated).async {
            let results = OpenCVWrapper.detectRedCircles(
                in: pixelBuffer,
                depthBuffer: depthMap,
                intrinsicsArray: intrinsicsArray
            ) as? [[String: NSNumber]]

            var lidar: LidarData = LidarData(lidarX: 0.0, lidarY: 0.0, lidarZ: 0.0)
            results?.forEach { dict in
                if let lx = dict["lidar_x"]?.doubleValue,
                   let ly = dict["lidar_y"]?.doubleValue,
                   let lz = dict["lidar_z"]?.doubleValue {
                    lidar = LidarData(lidarX: lx, lidarY: ly, lidarZ: lz)
                }
            }
            // 전송
            self.lidarSender.sendLidarPosition(
                x: lidar.lidarX,
                y: lidar.lidarY,
                z: lidar.lidarZ
            )
            print("버튼 클릭 시 LiDAR Coor: (\(lidar.lidarX), \(lidar.lidarY), \(lidar.lidarZ))")
        }
    }
    
    func sendCurrentLidarPosition() {
        self.lidarSender.sendLidarPosition(
            x: self.lidarPosition.lidarX,
            y: self.lidarPosition.lidarY,
            z: self.lidarPosition.lidarZ
        )
        print("Sent LiDAR Coor: (\(self.lidarPosition.lidarX), \(self.lidarPosition.lidarY), \(self.lidarPosition.lidarZ))")
    }

    func saveDepthMap() {
        guard let depthMap = latestDepthMap, let image = latestImage, let cameraIntrinsics = latestCameraIntrinsics else {
            print("Depth map or image is not available.")
            return
        }
        
        let documentsDir = FileManager.default.urls(for: .documentDirectory, in: .userDomainMask).first!
        let dateFormatter = DateFormatter()
        dateFormatter.dateFormat = "yyyyMMdd_with_parameters"
        let dateString = dateFormatter.string(from: Date())
        let dateDirURL = documentsDir.appendingPathComponent(dateString)
        
        do {
            try FileManager.default.createDirectory(at: dateDirURL, withIntermediateDirectories: true, attributes: nil)
        } catch {
            print("Failed to create directory: \(error)")
            return
        }
        
        let timestamp = Date().timeIntervalSince1970
        let depthFileURL = dateDirURL.appendingPathComponent("\(timestamp)_depth.tiff")
        let depthRawFileURL = dateDirURL.appendingPathComponent("\(timestamp)_depth.raw")
        let imageFileURL = dateDirURL.appendingPathComponent("\(timestamp)_image.jpg")
        let intrinsicsFileURL = dateDirURL.appendingPathComponent("\(timestamp)_intrinsics.dat")
            
        do {
            let intrinsicsData = cameraIntrinsics.toData()
            try intrinsicsData.write(to: intrinsicsFileURL)
            print("Intrinsics saved to \(intrinsicsFileURL)")
        } catch {
            print("Failed to save intrinsics: \(error)")
        }
        
        writeDepthMapToTIFFWithLibTIFF(depthMap: depthMap, url: depthFileURL)
        writeDepthMapToRawFile(depthMap: depthMap, url: depthRawFileURL)
        saveImage(image: image, url: imageFileURL)
        
        let uiImage = UIImage(ciImage: CIImage(cvPixelBuffer: image))
        
        
        DispatchQueue.main.async {
            self.lastCapture = uiImage
        }
     
        
        
        print("Depth map saved to \(depthFileURL)")
        print("Image saved to \(imageFileURL)")
    }
}

extension simd_float3x3 {
    func toData() -> Data {
        var matrix = self
        return Data(bytes: &matrix, count: MemoryLayout<simd_float3x3>.size)
    }
}
