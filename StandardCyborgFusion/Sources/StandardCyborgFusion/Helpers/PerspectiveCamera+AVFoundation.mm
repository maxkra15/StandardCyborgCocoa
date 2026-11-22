//
//  PerspectiveCamera+AVFoundation.mm
//  StandardCyborgFusion
//
//  Created by Aaron Thompson on 12/19/18.
//

#import <AVFoundation/AVFoundation.h>
#import <standard_cyborg/util/DataUtils.hpp>
#import <standard_cyborg/math/Mat3x3.hpp>
#import <standard_cyborg/math/Mat3x4.hpp>
#import <standard_cyborg/math/Vec2.hpp>
#import <vector>

#import "EigenHelpers.hpp"
#import "GeometryHelpers.hpp"
#import "PerspectiveCamera+AVFoundation.hpp"
#import <StandardCyborgFusion/SCPointCloud.h>
#include <cmath>
#include <limits>

using namespace standard_cyborg;

simd_float3x3 SCReconstructionCameraOrientationMatrix(void)
{
    return simd_matrix(simd_make_float3(0, 1, 0),
                       simd_make_float3(1, 0, 0),
                       simd_make_float3(0, 0, -1));
}

simd_float3 SCReconstructionGravityForDeviceGravity(
    simd_float3 deviceGravity, simd_float4x3 calibrationExtrinsic, BOOL flipsInputHorizontally)
{
    const float nan = std::numeric_limits<float>::quiet_NaN();
    const simd_float3 invalid = simd_make_float3(nan, nan, nan);
    if (!std::isfinite(deviceGravity.x) || !std::isfinite(deviceGravity.y) || !std::isfinite(deviceGravity.z)) {
        return invalid;
    }
    const float length = simd_length(deviceGravity);
    if (!std::isfinite(length) || length < 0.5f || length > 1.5f) { return invalid; }
    const simd_float3x3 rotation = simd_matrix(calibrationExtrinsic.columns[0],
                                              calibrationExtrinsic.columns[1],
                                              calibrationExtrinsic.columns[2]);
    for (int column = 0; column < 3; ++column) {
        for (int row = 0; row < 3; ++row) {
            if (!std::isfinite(rotation.columns[column][row])) { return invalid; }
        }
    }
    const simd_float3x3 gram = simd_mul(simd_transpose(rotation), rotation);
    for (int column = 0; column < 3; ++column) {
        for (int row = 0; row < 3; ++row) {
            if (std::abs(gram.columns[column][row] - (column == row ? 1.0f : 0.0f)) > 0.002f) {
                return invalid;
            }
        }
    }
    if (std::abs(simd_determinant(rotation) - 1.0f) > 0.002f) { return invalid; }
    const simd_float3x3 orientation = SCReconstructionCameraOrientationMatrix();
    // PerspectiveCamera uses V = R * O and transforms sensor rays by V^-1.
    // O also maps baseline Core Motion axes into the SDK's pre-view camera axes.
    // Apply that same basis change to gravity rather than treating a device
    // vector as if it were already in the model's initial camera coordinates.
    simd_float3 cameraGravity = simd_mul(orientation, deviceGravity / length);
    if (flipsInputHorizontally) { cameraGravity.y = -cameraGravity.y; }
    return simd_mul(simd_transpose(orientation), simd_mul(simd_transpose(rotation), cameraGravity));
}

sc3d::PerspectiveCamera PerspectiveCameraFromAVCameraCalibrationData(AVCameraCalibrationData *calibrationData, size_t pixelsWide, size_t pixelsHigh, float focalLengthScaleFactor)
{
    NSData *lensDistortionLookupTableData = calibrationData.lensDistortionLookupTable;
    NSData *inverseLensDistortionLookupTableData = calibrationData.inverseLensDistortionLookupTable;
    
    std::vector<float> lensDistortionTable {};
    const float* tableData = (float*)[lensDistortionLookupTableData bytes];
    if (tableData != nullptr) { lensDistortionTable.assign(tableData, tableData + [lensDistortionLookupTableData length] / sizeof(float)); }

    std::vector<float> inverseLensDistortionTable {};
    const float* inverseTableData = (float*)[inverseLensDistortionLookupTableData bytes];
    if (inverseTableData != nullptr) { inverseLensDistortionTable.assign(inverseTableData, inverseTableData + [inverseLensDistortionLookupTableData length] / sizeof(float)); }

    sc3d::PerspectiveCamera camera;

    // Determined after length investigation of the effect of focal length on dimensional accuracy and
    // loop closure. There's a fair bit of scatter in the data, so this is certainly not "correct",
    // merely "much, much better than nothing." For more details, see:
    //    https://github.com/StandardCyborg/ScanAnalysis/tree/master/ScanAnalysis/2019-05-09-dimensional-accuracy-calibration
    // Preserve the SDK default; metric callers opt into the supplied calibrated intrinsics.
    camera.setFocalLengthScaleFactor(focalLengthScaleFactor);

    math::Mat3x4 extrinsicMatrix(toMat3x4(calibrationData.extrinsicMatrix));
    camera.setExtrinsicMatrix(extrinsicMatrix);

    // The extrinsic matrix Apple gives us in calibrationData.extrinsicMatrix doesn't match the
    // coordinate system we otherwise prefer, so we use this matrix to rotate the model about the
    // z axis by 90 degrees so that the model is unprojected out along the negative z axis. This
    // means that if you look at the screen with x to the right and y up, then you see the model
    // in front of you.
    const simd_float3x3 outputOrientation = SCReconstructionCameraOrientationMatrix();
    math::Mat3x4 desiredOrientation({
        outputOrientation.columns[0].x, outputOrientation.columns[1].x, outputOrientation.columns[2].x, 0,
        outputOrientation.columns[0].y, outputOrientation.columns[1].y, outputOrientation.columns[2].y, 0,
        outputOrientation.columns[0].z, outputOrientation.columns[1].z, outputOrientation.columns[2].z, 0
    });
    // We construct a baseline extrinsic matrix strictly to nail down our desired output given
    // expected input, with the additional expectation that if the extrinsic matrix returned by
    // Apple changed for some reason, we'd actually want to pick up those changes. That is to say,
    // all of this is constructed to save and use Apple's extrinsic matrix but post-multiply it
    // by our own matrix strictly to achieve the matrix we *want*.
    math::Mat3x4 baselineExtrinsicMatrix = math::Mat3x4::Identity();
    math::Mat3x4 orientationMatrix(desiredOrientation * baselineExtrinsicMatrix.inverse());
    
    camera.setOrientationMatrix(orientationMatrix);
    
    camera.setNominalIntrinsicMatrix(toMat3x3(calibrationData.intrinsicMatrix));
    camera.setIntrinsicMatrixReferenceSize(math::Vec2(calibrationData.intrinsicMatrixReferenceDimensions.width,
                                                           calibrationData.intrinsicMatrixReferenceDimensions.height));
    camera.setLensDistortionCenter(math::Vec2(calibrationData.lensDistortionCenter.x, calibrationData.lensDistortionCenter.y));
    camera.setLensDistortionLookupTable(lensDistortionTable);
    camera.setInverseLensDistortionLookupTable(inverseLensDistortionTable);
    camera.setLegacyImageSize({(int)pixelsWide, (int)pixelsHigh});
    
    return camera;
}
