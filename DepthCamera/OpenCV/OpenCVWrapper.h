//
//  OpenCVWrapper.h
//  DepthCamera
//
//  Created by 이은화 on 5/20/25.
//

#import <Foundation/Foundation.h>
#import <CoreVideo/CoreVideo.h>
#import <ARKit/ARKit.h>

NS_ASSUME_NONNULL_BEGIN

@interface OpenCVWrapper : NSObject

+ (NSArray<NSDictionary *> *)detectRedCirclesInPixelBuffer:
(CVPixelBufferRef)pixelBuffer
                                               depthBuffer:
(CVPixelBufferRef)depthBuffer
                                           intrinsicsArray:
(const float *)intrinsicsArray
;

@end

NS_ASSUME_NONNULL_END
