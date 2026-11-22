//
//  SCPointCloud.h
//  StandardCyborgFusion
//
//

#import <Foundation/Foundation.h>
#import <simd/simd.h>

#ifdef __cplusplus
extern "C" {
#endif

/** The fixed orientation used by the SDK's calibrated depth unprojection. */
FOUNDATION_EXPORT simd_float3x3 SCReconstructionCameraOrientationMatrix(void);

/**
 Converts Core Motion hardware-frame gravity into the initial reconstruction
 camera frame, using the same rotation as depth unprojection. Translation is
 ignored because gravity is a direction. The historical input mirror option
 reflects raw image rows and is included in this mapping. Returns all-NaN for
 invalid gravity or a nonfinite/nonrigid calibration rotation.
 */
FOUNDATION_EXPORT simd_float3 SCReconstructionGravityForDeviceGravity(
    simd_float3 deviceGravity, simd_float4x3 calibrationExtrinsic, BOOL flipsInputHorizontally);

#ifdef __cplusplus
}
#endif

@interface SCPointCloud : NSObject

/** The interval between each point's interleaved position, normal, etc data */
+ (size_t)pointStride;

/** The offset within each interleaved stride interval of the start of each XYZ position */
+ (size_t)positionOffset;
+ (size_t)normalOffset;
+ (size_t)colorOffset;
+ (size_t)weightOffset;
+ (size_t)pointSizeOffset;

/** The number of components in an XYZ position (hint: it's 3) */
+ (size_t)positionComponentCount;
+ (size_t)normalComponentCount;
+ (size_t)colorComponentCount;
+ (size_t)weightComponentCount;
+ (size_t)pointSizeComponentCount;

/** The size, in bytes, of each position component. An XYZ position is positionComponentSize * positionComponentCount bytes long */
+ (size_t)positionComponentSize;
+ (size_t)normalComponentSize;
+ (size_t)colorComponentSize;
+ (size_t)weightComponentSize;
+ (size_t)pointSizeComponentSize;

/** The number of points within this cloud */
@property (nonatomic, readonly) NSInteger pointCount;

/** Buffer for all points in the cloud. Position, normal, and other data for each point are interleaved at the offsets provided. */
@property (nonatomic, readonly) NSData *pointsData;

@property (nonatomic, readonly) simd_float3 gravity;

/** Returns a cloud sharing the point buffer, with gravity in the same coordinate frame. */
- (SCPointCloud * _Nonnull)pointCloudBySettingGravity:(simd_float3)gravity NS_SWIFT_NAME(settingGravity(_:));

/** Computes the geometric mean of this point cloud */
- (simd_float3)centerOfMass;

- (instancetype)init NS_UNAVAILABLE;

@end
