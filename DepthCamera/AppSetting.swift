//
//  AppSetting.swift
//  DepthCamera
//
//  Created by 이은화 on 6/10/25.
//

import SwiftUI
import Combine

class AppSettings: ObservableObject {
    @Published var mode: Mode = .three
    @Published var captureCount: Int = 0 // 버튼 누른 횟수

    enum Mode: String, CaseIterable, Identifiable {
        case three, fourOne, fourColors, six
        var id: String { self.rawValue }
    }
}
