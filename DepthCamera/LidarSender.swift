//
//  LidarSender.swift
//  DepthCamera
//
//  Created by 이은화 on 5/20/25.
//
import Foundation
import Network

class LidarSender {
    let connection: NWConnection
    let host: String
    let port: UInt16

    init(host: String, port: UInt16) {
        self.host = host
        self.port = port
        self.connection = NWConnection(host: NWEndpoint.Host(host), port: NWEndpoint.Port(rawValue: port)!, using: .udp)
        self.connection.stateUpdateHandler = { newState in
            print("[LidarSender] Connection state: \(newState)")
        }
        self.connection.start(queue: .global())
        print("[LidarSender] Initialized for \(host):\(port)")
    }

    func sendLidarPosition(x: Double, y: Double, z: Double) {
        let msg = String(format: "lidar:%.3f,%.3f,%.3f\n", x, y, z)
        guard let data = msg.data(using: .utf8) else {
            print("[LidarSender] Failed to encode message")
            return
        }
        print("[LidarSender] Sending: \(msg.trimmingCharacters(in: .whitespacesAndNewlines))")
        self.connection.send(content: data, completion: .contentProcessed { error in
            if let error = error {
                print("[LidarSender] Send error: \(error)")
            } else {
                print("[LidarSender] Data sent successfully")
            }
        })
    }
    func sendPoseUDP(anchorPos: SIMD3<Float>, anchorOri: simd_quatf,
                     cameraPos: SIMD3<Float>, cameraOri: simd_quatf) {
        // 예시: CSV 포맷으로 변환
        let msg = String(format: "anchor,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f;camera,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
            anchorPos.x, anchorPos.y, anchorPos.z, anchorOri.vector.x, anchorOri.vector.y, anchorOri.vector.z, anchorOri.vector.w,
            cameraPos.x, cameraPos.y, cameraPos.z, cameraOri.vector.x, cameraOri.vector.y, cameraOri.vector.z, cameraOri.vector.w
        )
        print("msgggggggg: \(msg)")
        let data = msg.data(using: .utf8)!
        connection.send(content: data, completion: .contentProcessed({ error in
            if let error = error {
                print("UDP send error: \(error)")
            }
        }))
    }
    func sendAnchorCoordCameraPose(shape: String, cameraPos: SIMD3<Float>, cameraOri: simd_quatf) {
        // 예시: CSV 포맷으로 변환
        let msg = String(format: "shape:\(shape);cam_in_anchor:%.4f,%.4f,%.4f,%.4f,%.4f,%.4f,%.4f",
             cameraPos.x, cameraPos.y, cameraPos.z,
             cameraOri.vector.x, cameraOri.vector.y, cameraOri.vector.z, cameraOri.vector.w
        )
        print("msgggggggg: \(msg)")
        let data = msg.data(using: .utf8)!
        connection.send(content: data, completion: .contentProcessed({ error in
            if let error = error {
                print("UDP send error: \(error)")
            }
        }))
    }
    
    func sendPlaneEquation(a: Double, b: Double, c: Double, d: Double) {
       let msg = String(format: "plane:%.6f,%.6f,%.6f,%.6f\n", a, b, c, d)
       guard let data = msg.data(using: .utf8) else {
           print("[LidarSender] Failed to encode plane equation")
           return
       }
       print("[LidarSender] Sending plane: \(msg.trimmingCharacters(in: .whitespacesAndNewlines))")
       self.connection.send(content: data, completion: .contentProcessed { error in
           if let error = error {
               print("[LidarSender] Plane send error: \(error)")
           } else {
               print("[LidarSender] Plane data sent successfully")
           }
       })
   }
    func sendNormalVector(nx: Double, ny: Double, nz: Double) {
       let msg = String(format: "normal:%.6f,%.6f,%.6f\n", nx, ny, nz)
       guard let data = msg.data(using: .utf8) else {
           print("[LidarSender] Failed to encode plane equation")
           return
       }
       print("[LidarSender] Sending plane: \(msg.trimmingCharacters(in: .whitespacesAndNewlines))")
       self.connection.send(content: data, completion: .contentProcessed { error in
           if let error = error {
               print("[LidarSender] Plane send error: \(error)")
           } else {
               print("[LidarSender] Plane data sent successfully")
           }
       })
   }
    func sendCamera(x: Double, y: Double, z: Double) {
       let msg = String(format: "camera:%.6f,%.6f,%.6f\n", x, y, z)
       guard let data = msg.data(using: .utf8) else {
           print("[LidarSender] Failed to encode plane equation")
           return
       }
       print("[LidarSender] Sending plane: \(msg.trimmingCharacters(in: .whitespacesAndNewlines))")
       self.connection.send(content: data, completion: .contentProcessed { error in
           if let error = error {
               print("[LidarSender] Plane send error: \(error)")
           } else {
               print("[LidarSender] Plane data sent successfully")
           }
       })
   }
    func sendType(type: String){
        let msg = String(format: "type:\(type)\n")
        guard let data = msg.data(using: .utf8) else {
            print("[LidarSender] Failed to encode plane equation")
            return
        }
        print("[LidarSender] Sending plane: \(msg.trimmingCharacters(in: .whitespacesAndNewlines))")
        self.connection.send(content: data, completion: .contentProcessed { error in
            if let error = error {
                print("[LidarSender] Plane send error: \(error)")
            } else {
                print("[LidarSender] Plane data sent successfully")
            }
        })
    }
    func sendQuatd(w: Double, x: Double, y: Double, z: Double) {
        let msg = String(format: "quat:%.6f,%.6f,%.6f,%.6f\n", w, x, y, z)
        guard let data = msg.data(using: .utf8) else {
            print("[LidarSender] Failed to encode plane equation")
            return
        }
        print("[LidarSender] Sending plane: \(msg.trimmingCharacters(in: .whitespacesAndNewlines))")
        self.connection.send(content: data, completion: .contentProcessed { error in
            if let error = error {
                print("[LidarSender] Plane send error: \(error)")
            } else {
                print("[LidarSender] Plane data sent successfully")
            }
        })
    }
    func sendCameraFull(x: Double, y: Double, z: Double,nx: Double, ny: Double, nz: Double) {
       let msg = String(format: "camera:%.6f,%.6f,%.6f;%.6f,%.6f,%.6f\n", x, y, z, nx, ny,nz)
       guard let data = msg.data(using: .utf8) else {
           print("[LidarSender] Failed to encode plane equation")
           return
       }
       print("[LidarSender] Sending plane: \(msg.trimmingCharacters(in: .whitespacesAndNewlines))")
       self.connection.send(content: data, completion: .contentProcessed { error in
           if let error = error {
               print("[LidarSender] Plane send error: \(error)")
           } else {
               print("[LidarSender] Plane data sent successfully")
           }
       })
   }
}
