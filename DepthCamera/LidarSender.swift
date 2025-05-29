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
        let msg = String(format: "%.3f,%.3f,%.3f\n", x, y, z)
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
}
