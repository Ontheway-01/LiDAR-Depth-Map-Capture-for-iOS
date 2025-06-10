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
struct CameraPosition {
    let x: Double
    let y: Double
    let z: Double
}
struct LidarPosition {
    let x: Double
    let y: Double
    let z: Double
}
struct RotationMatrix {
    let r: [[Double]] // 3x3
}
struct PixelCenter {
    let x: Double
    let y: Double
    let radius: Double
}
struct NormalVector {
    let x: Double
    let y: Double
    let z: Double
}
struct PlaneEquation {
    let a: Double
    let b: Double
    let c: Double
    let d: Double
}
struct DetectionResult {
//    let cameraPosition: CameraPosition
    let lidarPosition: LidarPosition
    let normal: NormalVector
    let planeEquation: PlaneEquation
//    let rotationWorldToCamera: RotationMatrix
    let pixelCenters: [PixelCenter]
}
func parseDetectionResult(_ dict: [String: Any]) -> DetectionResult {
    // Lidar Position
    let lidarDict = dict["lidar_position"] as? [String: Any] ?? [:]
    let lidarPosition = LidarPosition(
        x: lidarDict["x"] as? Double ?? 0.0,
        y: lidarDict["y"] as? Double ?? 0.0,
        z: lidarDict["z"] as? Double ?? 0.0
    )

    let normal = dict["normal"] as? [String: Any] ?? [:]
    let normalVector = NormalVector(
        x: normal["x"] as? Double ?? 0.0,
        y: normal["y"] as? Double ?? 0.0,
        z: normal["z"] as? Double ?? 0.0
    )

    let plane = dict["plane_equation"] as? [String: Any] ?? [:]
    let planeEquation = PlaneEquation(
        a: plane["a"] as? Double ?? 0.0,
        b: plane["b"] as? Double ?? 0.0,
        c: plane["c"] as? Double ?? 0.0,
        d: plane["d"] as? Double ?? 0.0
    )

    // Pixel Centers
    let pixelArr = dict["pixel_centers"] as? [[String: Any]] ?? []
    let pixelCenters = pixelArr.map { p in
        PixelCenter(
            x: p["x"] as? Double ?? 0.0,
            y: p["y"] as? Double ?? 0.0,
            radius: p["radius"] as? Double ?? 0.0
        )
    }

    return DetectionResult(
        lidarPosition: lidarPosition,
        normal: normalVector,
        planeEquation: planeEquation,
        pixelCenters: pixelCenters
    )
}

class ARViewModel: NSObject, ARSessionDelegate, ObservableObject {
    let lidarSender = LidarSender(host: "192.168.1.216", port: 5005)
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
    private var planeEquation: PlaneEquation = PlaneEquation(a: 0.0, b: 0.0, c: 0.0, d: 0.0)
    private var cameraPosition: CameraPosition = CameraPosition(x: 0.0, y: 0.0, z: 0.0)
    @Published var pixelBufferWidth: Int?
    @Published var pixelBufferHeight: Int?
    var appSettings: AppSettings? // 옵셔널로 보관

    func setAppSettings(_ settings: AppSettings) {
        self.appSettings = settings
    }
    
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
        
        let cameraPose_world: simd_float4x4 = frame.camera.transform
        let cameraMatrix: [Float] = [
            cameraPose_world.columns.0.x, cameraPose_world.columns.0.y, cameraPose_world.columns.0.z, cameraPose_world.columns.0.w,
            cameraPose_world.columns.1.x, cameraPose_world.columns.1.y, cameraPose_world.columns.1.z, cameraPose_world.columns.1.w,
            cameraPose_world.columns.2.x, cameraPose_world.columns.2.y, cameraPose_world.columns.2.z, cameraPose_world.columns.2.w,
            cameraPose_world.columns.3.x, cameraPose_world.columns.3.y, cameraPose_world.columns.3.z, cameraPose_world.columns.3.w,
        ]
        let lidarOffset = simd_float4(0, 0, 0, 1) // 필요시 실제 오프셋 값 사용

        // 라이다 월드 좌표 = 카메라 월드 좌표 + 오프셋 (회전 포함)
        let lidarWorldPos4 = cameraPose_world * lidarOffset
        let lidarWorldPos = SIMD3<Float>(lidarWorldPos4.x, lidarWorldPos4.y, lidarWorldPos4.z)
        let lidarWorldArray: [Float] = [lidarWorldPos.x, lidarWorldPos.y, lidarWorldPos.z]
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
            let results = OpenCVWrapper.detectRedCircles(in: pixelBuffer, depthBuffer: depthMap, intrinsicsArray: intrinsicsArray, cameraMatrix: cameraMatrix, lidarWorldArray: lidarWorldArray) as? [[String: NSNumber]]
            print(results)
            var circles: [CircleData] = []
            var lidar: LidarData = LidarData(lidarX: 0.0, lidarY: 0.0, lidarZ: 0.0)
            var plane: PlaneEquation = PlaneEquation(a: 0.0, b: 0.0, c: 0.0, d: 0.0)
            var cameraPos: CameraPosition = CameraPosition(x: 0.0, y: 0.0, z: 0.0)

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
                   let lz = dict["lidar_z"]?.doubleValue,
                   let ca = dict["camera_a"]?.doubleValue,
                   let cb = dict["camera_b"]?.doubleValue,
                   let cc = dict["camera_c"]?.doubleValue,
                   let cd = dict["camera_d"]?.doubleValue,
                   let cx = dict["camera_x"]?.doubleValue,
                   let cy = dict["camera_y"]?.doubleValue,
                   let cz = dict["camera_z"]?.doubleValue
                {
                    let circle = CircleData(pixelX: px, pixelY: py, radius: radius, depth: depth, triangleX: tx, triangleY: ty, triangleZ: tz)
                    lidar = LidarData(lidarX: lx, lidarY: ly, lidarZ: lz)
                    circles.append(circle)
                    plane = PlaneEquation(a: ca, b: cb, c: cc, d: cd)
                    cameraPos = CameraPosition(x: cx, y: cy, z: cz)
                }
            }

            DispatchQueue.main.async {
                self.detectedCircles = circles
                self.lidarPosition = lidar
                self.planeEquation = plane
                self.cameraPosition = cameraPos
            }
        }
    }

    func sendCurrentLidarPosition() {
        print("send!!!!!!!!!!!!!!!!!!!!!!!")
        self.lidarSender.sendLidarPosition(x: self.lidarPosition.lidarX, y: self.lidarPosition.lidarY, z: self.lidarPosition.lidarZ)
        self.lidarSender.sendNormalVector(nx: self.planeEquation.a, ny: self.planeEquation.b, nz: self.planeEquation.c)
        self.lidarSender.sendCamera(x: self.cameraPosition.x, y: self.cameraPosition.y, z: self.cameraPosition.z)
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
