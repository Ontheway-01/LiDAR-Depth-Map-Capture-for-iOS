//
//  ARViewModel.swift
//  DepthCamera
//
//  Created by iori on 2024/11/27.
//

import ARKit


class ARViewModel: NSObject, ARSessionDelegate, ObservableObject {
    private var latestDepthMap: CVPixelBuffer?
    private var latestImage: CVPixelBuffer?
    private var latestCameraIntrinsics: simd_float3x3?
    @Published var lastCapture: UIImage? = nil {
        didSet {
            print("lastCapture was set.")
        }
    }
    
    
    func session(_ session: ARSession, didUpdate frame: ARFrame) {
        latestCameraIntrinsics = frame.camera.intrinsics

        print("RGB: \(latestCameraIntrinsics)")
        
        latestDepthMap = frame.sceneDepth?.depthMap
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
