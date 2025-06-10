//
//  DepthCameraApp.swift
//  DepthCamera
//
//  Created by iori on 2023/11/27.
//

import SwiftUI

@main
struct DepthCameraApp: App {
    @StateObject private var settings = AppSettings()

    var body: some Scene {
        WindowGroup {
            ContentView()
            .environmentObject(settings)        }
    }
}
