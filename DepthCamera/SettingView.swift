//
//  SettingView.swift
//  DepthCamera
//
//  Created by 이은화 on 6/10/25.
//
import SwiftUI

struct SettingView: View {
    @EnvironmentObject var settings: AppSettings
    @ObservedObject var arViewModel: ARViewModel

    var body: some View {
        NavigationStack {
            VStack{
                Form {
                    Section(header: Text("컬러 모드")) {
                        Picker("Color Mode", selection: $settings.mode) {
                            ForEach(AppSettings.Mode.allCases) { mode in
                                Text(mode.rawValue).tag(mode)
                            }
                        }
                        .pickerStyle(SegmentedPickerStyle())
                        .onChange(of: settings.mode) { _ in
                            settings.captureCount = 0
                        }
                    }
                    Section(header: Text("캡처 카운트")) {
                        HStack {
                            Text("캡처 횟수: \(settings.captureCount)")
                            Spacer()
                            Button("초기화") {
                                settings.captureCount = 0
                            }
                        }
                    }
                }
                Button("설정 변경") {
                    arViewModel.setAppSettings(settings)
                }
            }
            .navigationTitle("설정")
        }
    }
}
