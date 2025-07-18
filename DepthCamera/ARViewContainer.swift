//
//  ARViewContainer.swift
//  DepthCamera
//
//  Created by iori on 2024/11/27.
//

import SwiftUI
import ARKit
import RealityKit


struct ARViewContainer: UIViewRepresentable {
    @ObservedObject var arViewModel: ARViewModel
    @EnvironmentObject var appSettings: AppSettings
    
    func find4by3VideoFormat() -> ARConfiguration.VideoFormat? {
        let availableFormats = ARWorldTrackingConfiguration.supportedVideoFormats
        for format in availableFormats {
            let resolution = format.imageResolution
            if resolution.width / 4 == resolution.height / 3 {
                print("Using video format: \(format)")
                return format
            }
        }
        return nil
    }
    
    
    
    func makeUIView(context: Context) -> ARView {
        let arView = ARView(frame: .zero)
        
        let configuration = ARWorldTrackingConfiguration()
//        configuration.worldAlignment = .camera
        
        if ARWorldTrackingConfiguration.supportsSceneReconstruction(.meshWithClassification) {
            configuration.sceneReconstruction = .meshWithClassification
        }
        
        if ARWorldTrackingConfiguration.supportsFrameSemantics(.sceneDepth) {
                   configuration.frameSemantics.insert(.sceneDepth)
               }
        
        if let format = find4by3VideoFormat() {
            configuration.videoFormat = format
        } else {
            print("No 4:3 video format is available")
        }
        
        
        arView.session.delegate = arViewModel
        arView.session.run(configuration)
        
        return arView
    }
    
    func updateUIView(_ uiView: ARView, context: Context) {
        let viewSize = uiView.bounds.size
        guard viewSize.width > 0, viewSize.height > 0 else { return }
        guard let bufferWidth = arViewModel.pixelBufferWidth,
              let bufferHeight = arViewModel.pixelBufferHeight else { return }
        let scaleX = viewSize.width / CGFloat(bufferHeight)
        let scaleY = viewSize.height / CGFloat(bufferWidth)
        print("bufferWidth: \(bufferWidth) / bufferHeight \(bufferHeight)")
        
        uiView.layer.sublayers?.removeAll(where: { $0.name == "circleOverlay" })
        uiView.layer.sublayers?.removeAll(where: { $0.name == "textOverlay" })

//        for (index, circle) in arViewModel.detectedCircles.enumerated() {
        for (index, circle) in arViewModel.detectedCircles.enumerated() {
            let textLayer = CATextLayer()
                textLayer.name = "textOverlay"
                textLayer.string = "\(index + 1)"
                textLayer.fontSize = 18
                textLayer.foregroundColor = UIColor.systemGreen.cgColor
                textLayer.alignmentMode = .center
                textLayer.contentsScale = UIScreen.main.scale
            let labelWidth: CGFloat = 24
            let labelHeight: CGFloat = 24

            let overlay = CAShapeLayer()
            overlay.name = "circleOverlay"
            let centerX = (CGFloat(bufferHeight) - CGFloat(circle.pixelY)) * scaleX
            let centerY = CGFloat(circle.pixelX) * scaleY

            let radius = CGFloat(circle.radius) * ((scaleX + scaleY) / 2)
            
            let rect = CGRect(x: centerX - radius,
                              y: centerY - radius,
                              width: radius * 2,
                              height: radius * 2)
            textLayer.frame = CGRect(
                x: centerX - radius,
                y: centerY - radius,
                width: labelWidth,
                height: labelHeight
            )
            overlay.path = UIBezierPath(ovalIn: rect).cgPath
            overlay.strokeColor = UIColor.systemGreen.cgColor
            overlay.fillColor = UIColor.clear.cgColor
            overlay.lineWidth = 3.0
            
            textLayer.backgroundColor = UIColor.white.withAlphaComponent(0.7).cgColor
            textLayer.cornerRadius = labelWidth / 2
            textLayer.masksToBounds = true
            
            uiView.layer.addSublayer(overlay)
            uiView.layer.addSublayer(textLayer)

        }
    }

}



