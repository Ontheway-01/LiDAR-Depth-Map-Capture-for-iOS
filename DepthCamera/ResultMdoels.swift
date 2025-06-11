//
//  ResultMdoels.swift
//  DepthCamera
//
//  Created by 이은화 on 6/11/25.
//

struct DetectionModels{
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

}
