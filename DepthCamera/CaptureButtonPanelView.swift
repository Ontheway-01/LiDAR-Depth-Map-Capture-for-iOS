//
//  CaptureButtonPanelView.swift
//  DepthCamera
//
//  Created by iori on 2024/11/27.
//

import SwiftUICore
import SwiftUI

struct CaptureButtonPanelView: View {
    @ObservedObject var model: ARViewModel
    var width: CGFloat
    @Binding var showSettings: Bool

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
                Spacer()
                ZStack(alignment: .bottomTrailing) {
                    Button(action: {
                       showSettings = true
                   }) {
                       Image(systemName: "gearshape")
                       Text("설정")
                   }
                   .frame(width: width / 3)
                   .padding(.horizontal)
                }
                
            }
            
        }
    }
    
    
}




