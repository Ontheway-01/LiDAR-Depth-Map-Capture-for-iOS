//
//  ARViewModel.swift
//  DepthCamera
//
//  Created by iori on 2024/11/27.
//

import ARKit
import SwiftUI

class ARViewModel: NSObject, ARSessionDelegate, ObservableObject {
//    let lidarSender = LidarSender(host: "192.168.1.216", port: 5005)
    let lidarSender = LidarSender(host: "192.168.0.57", port: 5005)
//    let lidarSender = LidarSender(host: "192.168.0.8", port: 5005)
    private var latestDepthMap: CVPixelBuffer?
    private var latestImage: CVPixelBuffer?
    private var latestCameraIntrinsics: simd_float3x3?
    @Published var lastCapture: UIImage? = nil {
        didSet {
            print("lastCapture was set.")
        }
    }
    
    @Published var detectedCircles: [DetectionModels.CircleData] = []
    private var lidarPosition: DetectionModels.LidarData = DetectionModels.LidarData(lidarX: 0.0, lidarY: 0.0, lidarZ: 0.0)
    private var planeEquation: DetectionModels.PlaneEquation = DetectionModels.PlaneEquation(a: 0.0, b: 0.0, c: 0.0, d: 0.0)
    private var cameraPosition: DetectionModels.CameraPosition = DetectionModels.CameraPosition(x: 0.0, y: 0.0, z: 0.0)
    private var quaternion: DetectionModels.Quaternion = DetectionModels.Quaternion(quatW: 0.0, quatX: 0.0, quatY: 0.0, quatZ: 0.0)

//    private var anchorPos: SIMD3<Float> = SIMD3<Float>(0.0,0.0,0.0)
//    private var cameraPos: SIMD3<Float> = SIMD3<Float>(0.0,0.0,0.0)
//    private var cameraPose: simd_quatf = simd_quatf(vector: SIMD4<Float>(0.0,0.0,0.0,0.0))
//    private var anchorPose: simd_quatf = simd_quatf(vector: SIMD4<Float>(0.0,0.0,0.0,0.0))
    private var camera_world_pose: simd_float4x4 = simd_float4x4()
    private var camera_tri_pos: SIMD3<Float> = SIMD3<Float>(0.0,0.0,0.0)
    private var camera_tri_ori: simd_quatf = simd_quatf(vector: SIMD4<Float>(0.0,0.0,0.0,0.0))
    @Published var pixelBufferWidth: Int?
    @Published var pixelBufferHeight: Int?
    private var shape: String = "three"
    var appSettings: AppSettings? // 옵셔널로 보관

    func setAppSettings(_ settings: AppSettings) {
        self.appSettings = settings
        if self.appSettings?.mode == .no{
            self.shape = "no"
        }else if self.appSettings?.mode == .three {
            self.shape = "three"
        }else if self.appSettings?.mode == .four {
            self.shape = "four"
        }else if self.appSettings?.mode == .six {
            self.shape = "six"
        }
    }
    
    private var lastProcessedTimestamp: TimeInterval = 0
    // 원하는 샘플링 간격(초): 0.05는 20fps, 0.1은 10fps
    private let minInterval: TimeInterval = 0.05
    
    func session(_ session: ARSession, cameraDidChangeTrackingState camera: ARCamera) {
        switch camera.trackingState {
        case .normal:
            print("정상 트래킹 중")
            if let transform = session.currentFrame?.camera.transform {
                print("카메라 위치: \(transform.columns.3.x), \(transform.columns.3.y), \(transform.columns.3.z)")
            }
        case .limited(let reason):
            print("제한된 트래킹: \(reason)")
        case .notAvailable:
            print("트래킹 불가")
        }
    }

    
    func session(_ arsession: ARSession, didUpdate frame: ARFrame) {
//        print("count::::::: \(self.appSettings?.captureCount)")
        let currentTimestamp = frame.timestamp
        if currentTimestamp - lastProcessedTimestamp < minInterval {
            return // 샘플링 간격보다 빠르면 처리하지 않음
        }


        lastProcessedTimestamp = currentTimestamp
        
        let pixelBuffer = frame.capturedImage
        let width = CVPixelBufferGetWidth(pixelBuffer)
        let height = CVPixelBufferGetHeight(pixelBuffer)

        latestCameraIntrinsics = frame.camera.intrinsics
        
//        print("RGB: \(latestCameraIntrinsics)")
        
        let cameraPose_world: simd_float4x4 = frame.camera.transform
        camera_world_pose = cameraPose_world
        let cameraMatrix: [Float] = [
            cameraPose_world.columns.0.x, cameraPose_world.columns.0.y, cameraPose_world.columns.0.z, cameraPose_world.columns.0.w,
            cameraPose_world.columns.1.x, cameraPose_world.columns.1.y, cameraPose_world.columns.1.z, cameraPose_world.columns.1.w,
            cameraPose_world.columns.2.x, cameraPose_world.columns.2.y, cameraPose_world.columns.2.z, cameraPose_world.columns.2.w,
            cameraPose_world.columns.3.x, cameraPose_world.columns.3.y, cameraPose_world.columns.3.z, cameraPose_world.columns.3.w,
        ]
        //그냥 transorm 구해서 저장
        
        //앵커 설정
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
            var results:[[String: Any]]?
            if (self.appSettings?.mode == .three){
                results = OpenCVWrapper.detectTriangleRedCircles(in: pixelBuffer, depthBuffer: depthMap, intrinsicsArray: intrinsicsArray, cameraMatrix: cameraMatrix, lidarWorldArray: lidarWorldArray) as? [[String: Any]]
//                print("result:: \(results)")
            }
            else if (self.appSettings?.mode == .four){
                results = OpenCVWrapper.detectRectangleRedCircles(in: pixelBuffer, depthBuffer: depthMap, intrinsicsArray: intrinsicsArray, cameraMatrix: cameraMatrix, lidarWorldArray: lidarWorldArray) as? [[String: Any]]
//                print(results)
            }
            else if (self.appSettings?.mode == .six){
                results = OpenCVWrapper.detectHexRedCircles(in: pixelBuffer, depthBuffer: depthMap, intrinsicsArray: intrinsicsArray, cameraMatrix: cameraMatrix, lidarWorldArray: lidarWorldArray) as? [[String: Any]]
//                print(results)
            }
            
            var circles: [DetectionModels.CircleData] = []
            var lidar: DetectionModels.LidarData = DetectionModels.LidarData(lidarX: 0.0, lidarY: 0.0, lidarZ: 0.0)
            var plane: DetectionModels.PlaneEquation = DetectionModels.PlaneEquation(a: 0.0, b: 0.0, c: 0.0, d: 0.0)
            var cameraPos: DetectionModels.CameraPosition = DetectionModels.CameraPosition(x: 0.0, y: 0.0, z: 0.0)
            var quat: DetectionModels.Quaternion = DetectionModels.Quaternion(quatW: 0.0, quatX: 0.0, quatY: 0.0, quatZ: 0.0)
            var camerapos: SIMD3<Float> = SIMD3<Float>(0.0,0.0,0.0)
            var cameraori: simd_quatf = simd_quatf(vector: SIMD4<Float>(0.0,0.0,0.0,0.0))
            
            results?.forEach { dict in
                if let px = dict["pixel_x"] as? NSNumber,
                   let py = dict["pixel_y"] as? NSNumber,
                   let radius = dict["radius"] as? NSNumber,
                   let depth = dict["depth"] as? NSNumber,
                   let lx = dict["lidar_x"] as? NSNumber,
                   let ly = dict["lidar_y"] as? NSNumber,
                   let lz = dict["lidar_z"] as? NSNumber,
                   let ca = dict["camera_a"] as? NSNumber,
                   let cb = dict["camera_b"] as? NSNumber,
                   let cc = dict["camera_c"] as? NSNumber,
                   let cd = dict["camera_d"] as? NSNumber,
                   let cx = dict["camera_x"] as? NSNumber,
                   let cy = dict["camera_y"] as? NSNumber,
                   let cz = dict["camera_z"] as? NSNumber,
                   let qw = dict["quat_w"] as? NSNumber,
                   let qx = dict["quat_x"] as? NSNumber,
                   let qy = dict["quat_y"] as? NSNumber,
                   let qz = dict["quat_z"] as? NSNumber,
                   let rotationArray = dict["rotation"] as? [NSNumber],
                   let translationArray = dict["translation"] as? [NSNumber]
                {
                    let circle = DetectionModels.CircleData(pixelX: px.doubleValue, pixelY: py.doubleValue, radius: radius.doubleValue, depth: depth.doubleValue)
                    lidar = DetectionModels.LidarData(lidarX: lx.doubleValue, lidarY: ly.doubleValue, lidarZ: lz.doubleValue)
                    circles.append(circle)
                    plane = DetectionModels.PlaneEquation(a: ca.doubleValue, b: cb.doubleValue, c: cc.doubleValue, d: cd.doubleValue)
                    cameraPos = DetectionModels.CameraPosition(x: cx.doubleValue, y: cy.doubleValue, z: cz.doubleValue)
                    quat = DetectionModels.Quaternion(quatW: qw.doubleValue, quatX: qx.doubleValue, quatY: qy.doubleValue, quatZ: qz.doubleValue)
                    let rotationFloats = rotationArray.map { $0.doubleValue }
                    let translationFloats = translationArray.map { $0.doubleValue }
//                    print("rotation floatts: \(rotationFloats)")
//                    print("calculate anchorr: \(translationFloats)")
                    
//                    print("anchor settingggggg!!!!!!!!!!!!")
                    
                    // simd_float4x4로 변환
                    
                    let R = simd_float3x3(rows: [
                        SIMD3<Float>(Float(rotationArray[0]), Float(rotationArray[1]), Float(rotationArray[2])),
                        SIMD3<Float>(Float(rotationArray[3]), Float(rotationArray[4]), Float(rotationArray[5])),
                        SIMD3<Float>(Float(rotationArray[6]), Float(rotationArray[7]), Float(rotationArray[8]))
                    ])
//                    print("RRRR:: \(R)")
                    let t = SIMD3<Float>(Float(translationArray[0]), Float(translationArray[1]), Float(translationArray[2]))
//                    print("anchor translationnnn:: \(translationFloats)")

                    // 4x4 변환행렬 생성
                    var transform = matrix_identity_float4x4
                    transform.columns.0 = SIMD4<Float>(R.columns.0, 0)
                    transform.columns.1 = SIMD4<Float>(R.columns.1, 0)
                    transform.columns.2 = SIMD4<Float>(R.columns.2, 0)
                    transform.columns.3 = SIMD4<Float>(t, 1)
//                    print(transform)
//                    let cameraInAnchor = transform.inverse * self.camera_world_pose
//                    camerapos = SIMD3<Float>(cameraInAnchor.columns.3.x,
//                                                cameraInAnchor.columns.3.y,
//                                                cameraInAnchor.columns.3.z)
//                    cameraori = simd_quatf(cameraInAnchor)
                    if self.appSettings!.captureCount == 0{
                        for anchor in frame.anchors {
                            arsession.remove(anchor: anchor)
                        }
//                        if self.appSettings?.mode == .three{
//                            let anchor = ARAnchor(name: "three", transform: transform)
//                            arsession.add(anchor: anchor)
//                        }
//                        if self.appSettings?.mode == .four{
//                            print("anchor four")
//                            let anchor = ARAnchor(name: "four", transform: transform)
//                            arsession.add(anchor: anchor)
//                        }
//                        if self.appSettings?.mode == .six{
//                            let anchor = ARAnchor(name: "six", transform: transform)
//                            arsession.add(anchor: anchor)
//                        }
                    }
                }
            }
//            else if self.appSettings?.mode == .no {
//                camerapos = SIMD3<Float>(self.camera_world_pose.columns.3.x,
//                                         self.camera_world_pose.columns.3.y,
//                                         self.camera_world_pose.columns.3.z)
//                cameraori = simd_quatf(self.camera_world_pose)
//            }
//            if self.appSettings!.captureCount >= 1 {
//                if self.appSettings?.mode == .three {
//                    for anchor in frame.anchors {
//                        if anchor.name == "triangle" {
//                            let transform = anchor.transform // simd_float4x4
//                            let cameraInAnchor = transform.inverse * self.camera_world_pose
//                            print("transformmm: \(transform) // camera transformm:: \(self.camera_world_pose)")
//                            camerapos = SIMD3<Float>(cameraInAnchor.columns.3.x,
//                                                        cameraInAnchor.columns.3.y,
//                                                        cameraInAnchor.columns.3.z)
//                            cameraori = simd_quatf(cameraInAnchor)
//                        }
//                    }
//                }
//            }
            DispatchQueue.main.async {
                self.detectedCircles = circles
                self.lidarPosition = lidar
                self.planeEquation = plane
                self.cameraPosition = cameraPos
                self.quaternion = quat
                self.camera_tri_pos = camerapos
                self.camera_tri_ori = cameraori
//                print("shapeee: \(self.shape)")
//                self.sendCurrentLidarPosition()
//                print("positionssss:: \(self.camera_tri_pos) // oriiii: \(self.camera_tri_ori)")
            }
        }
        
    }
    func clamp<T: Comparable>(_ value: T, min minValue: T, max maxValue: T) -> T {
        return max(min(value, maxValue), minValue)
    }

    func applyGravityCorrection(
        transform: simd_float4x4,
        gravity: SIMD3<Float>
    ) -> simd_float4x4 {
        let currentY = normalize(simd_float3(transform.columns.1.x,
                                             transform.columns.1.y,
                                             transform.columns.1.z))
        let gravityDir = normalize(gravity)
        let rotationAxis = cross(currentY, gravityDir)
        let axisNorm = length(rotationAxis)
        // 이미 정렬되어 있으면 보정 불필요
        if axisNorm < 1e-5 { return transform }
        let rotationAxisNorm = rotationAxis / axisNorm
        let cosAngle = dot(currentY, gravityDir)
        let clampedCosAngle = clamp(cosAngle, min: -1.0, max: 1.0)
        let rotationAngle = acos(clampedCosAngle)
        let rotationQuat = simd_quatf(angle: rotationAngle, axis: rotationAxisNorm)
        let rotationMatrix = simd_float4x4(rotationQuat)
        return rotationMatrix * transform
    }

    // 3. 핸드폰 하단부 위치 계산
    func calculatePhoneBottomPosition(
        cameraTransform: simd_float4x4,
        offsetFromCameraToCenter: SIMD3<Float> = [0, 0, -0.02],
        offsetFromCenterToBottom: SIMD3<Float> = [0, -0.15, 0]
    ) -> SIMD3<Float> {
        let cameraPosition = simd_float3(cameraTransform.columns.3.x,
                                         cameraTransform.columns.3.y,
                                         cameraTransform.columns.3.z)
        let rotation = simd_float3x3(
            simd_float3(cameraTransform.columns.0.x, cameraTransform.columns.0.y, cameraTransform.columns.0.z),
            simd_float3(cameraTransform.columns.1.x, cameraTransform.columns.1.y, cameraTransform.columns.1.z),
            simd_float3(cameraTransform.columns.2.x, cameraTransform.columns.2.y, cameraTransform.columns.2.z)
        )
        let offsetCenter = rotation * offsetFromCameraToCenter
        let offsetBottom = rotation * offsetFromCenterToBottom
        return cameraPosition + offsetCenter + offsetBottom
    }


    // 4. 메인 업데이트 로직
//    func updatePhonePosition() {
//        guard let frame = session.currentFrame else { return }
//        // 카메라 포즈 가져오기
//        var cameraTransform = frame.camera.transform
//        // 중력 보정 적용
//        cameraTransform = applyGravityCorrection(
//            transform: cameraTransform,
//            gravity: frame.gravity
//        )
//        // 하단부 위치 계산
//        let bottomPosition = calculatePhoneBottomPosition(
//            cameraTransform: cameraTransform
//        )
//        print("하단부 위치: \(bottomPosition)")
//    }
    
    func sendCurrentLidarPosition() {
//        print("send!!!!!!!!!!!!!!!!!!!!!!!")
//        self.lidarSender.sendAnchorCoordCameraPose(shape: shape, cameraPos: camera_tri_pos, cameraOri: camera_tri_ori)
//        self.lidarSender.sendPoseUDP(anchorPos: self.anchorPos, anchorOri: self.anchorPose, cameraPos: self.cameraPos, cameraOri: self.cameraPose)
        if (self.appSettings?.mode == .three){
            self.lidarSender.sendType(type: "three")
        }
        else if (self.appSettings?.mode == .four){
            self.lidarSender.sendType(type: "four")
        }
        else if (self.appSettings?.mode == .six){
            self.lidarSender.sendType(type: "six")
        }
//        self.lidarSender.sendLidarPosition(x: self.lidarPosition.lidarX, y: self.lidarPosition.lidarY, z: self.lidarPosition.lidarZ)
        self.lidarSender.sendNormalVector(nx: self.planeEquation.a, ny: self.planeEquation.b, nz: self.planeEquation.c)
        self.lidarSender.sendCamera(x: self.cameraPosition.x, y: self.cameraPosition.y, z: self.cameraPosition.z)
        self.lidarSender.sendQuatd(w: self.quaternion.quatW, x: self.quaternion.quatX, y: self.quaternion.quatY, z: self.quaternion.quatZ)
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
