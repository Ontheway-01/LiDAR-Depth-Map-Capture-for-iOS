//
//  CaptureButtonPanelView.swift
//  DepthCamera
//
//  Created by iori on 2024/11/27.
//

import SwiftUICore


struct CaptureButtonPanelView: View {
    @ObservedObject var model: ARViewModel
    var width: CGFloat
    @Environment(\.presentationMode) var presentationMode
    @State private var showAlert = false // State variable to control alert visibility
    
    
    
    
    
    
    var body: some View {
        ZStack(alignment: .bottomLeading) {
            HStack {
                ZStack(alignment: .topTrailing) {
                    ThumbnailView( model: model)
                        .frame(width: width / 3)
                        .padding(.horizontal)
                }
                Spacer()
            }
            HStack {
                Spacer()
                CaptureButton(model: model)
                Spacer()
            }
            HStack {
                
            }
        }
    }
    
    
}




