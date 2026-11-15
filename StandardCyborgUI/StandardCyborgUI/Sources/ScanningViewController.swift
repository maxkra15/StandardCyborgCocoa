import AVFoundation
import CoreMotion
import StandardCyborgFusion
import UIKit

@objc public protocol ScanningViewControllerDelegate: AnyObject {
    func scanningViewControllerDidCancel(_ controller: ScanningViewController)
    @objc optional func scanningViewController(_ controller: ScanningViewController, didScan pointCloud: SCPointCloud)
    @objc optional func scanningViewController(_ controller: ScanningViewController, didFail message: String)
    @objc optional func scanningViewController(_ controller: ScanningViewController, trackingIsStable: Bool)
    @objc optional func scanningViewController(_ controller: ScanningViewController, didWriteDiagnostics url: URL)
    @objc optional func scanningViewController(_ controller: ScanningViewController, captureGuidance message: String?)
}

/**
    Shows a live color + depth camera preview and shutter button.
 
    When the shutter is tapped, performs a customizable 3-second
    countdown, then starts scanning.
 
    When scanning is manually finished, or if it fails,
    reconstructs a 3D point cloud and informs its delegate.
 
    This class does not itself show a preview of the scan.
 
    Rendering can be customized by setting the scanningViewRenderer
    to your own object conforming to that protocol.
 */
@objc open class ScanningViewController: UIViewController,
    CameraManagerDelegate,
    SCReconstructionManagerDelegate
{
    
    // MARK: - Public

    /// Opt-in metric capture: sample gravity in the initial camera coordinate frame.
    public var usesDeviceMotionGravity = false
    /// Set before loading the view. Legacy SDK users retain their historic calibration.
    public var cameraFocalLengthScaleFactor: Float = 1.019
    /// Delivered on main; describes accepted capture effort, never foot completeness.
    public var onCaptureProgress: ((ScanCaptureProgress) -> Void)?
    private var _captureLifetime = ScanCaptureLifetime()
    /// Zero retains the SDK's center-weighted cutoff. Metres otherwise.
    public var maximumReconstructionDepth: Float = 0
    /// Allow rejected frames to recover against the last good model before ending.
    public var trackingRecoveryTime: TimeInterval = 0
    private let _motionManager = CMMotionManager()
    private var _captureState = ScanCaptureState()
    public var recordsScanDiagnostics = false
    public var reconstructionIterationLimit: Int32 = 0
    public var minimumCaptureDistance: Float = 0
    public var maximumCaptureDistance: Float = 0.5
    private var _depthGuidance: ScanDepthGuidance?
    private var _lastDepthGuidanceTime: TimeInterval = 0
    private var _lastReportedDepthMessage: String?
    private var _lastDepthMessageTime: TimeInterval = 0
    private var _diagnostics: ScanDiagnosticsRecorder?
    private var _isCompletingCapture = false
    
    @objc public enum ScanningTerminationReason: Int {
        case canceled
        case finished
    }
    
    @objc public weak var delegate: ScanningViewControllerDelegate?
    
    /** Override to drop in your own visualization */
    @objc public lazy var scanningViewRenderer: ScanningViewRenderer =
        DefaultScanningViewRenderer(device: _metalDevice, commandQueue: _visualizationCommandQueue)
    
    /** The duration of each count in the pre-scan countdown after tapping the shutter button */
    @objc public var countdownPerSecondDuration = 0.75
    
    /** The count of the pre-scan countdown after tapping the shutter button. Set to 0 to disable the countdown. */
    @objc public var countdownStartCount = 3
    
    /** You may customize the dismiss button by setting its public properties, or by hiding it and adding your own */
    @objc public let dismissButton = UIButton()
    
    /** You may customize the shutter button by setting ShutterButton's public properties, or by hiding it and adding your own */
    @objc public let shutterButton = ShutterButton()
    
    /** A convenience initializer that simply calls init() and sets the delegate */
    @objc public convenience init(delegate: ScanningViewControllerDelegate) {
        self.init()
        self.delegate = delegate
    }
    
    @objc public func shutterTapped(_ sender: UIButton?) {
        guard
            presentedViewController == nil
            else { return }
        
        switch _state {
        case .default:
            guard _cameraManager.isSessionRunning else { return }
            _startCountdown { self.startScanning() }
        case .countdownSeconds(let seconds):
            if seconds > 0 {
                ScanningHapticFeedbackEngine.shared.scanningCanceled()
                _cancelCountdown()
            }
        case .scanning:
            ScanningHapticFeedbackEngine.shared.scanningFinished()
            stopScanning(reason: .finished)
        }
    }
    
    /** Starts scanning immediately */
    @objc public func startScanning() {
        guard !_isCompletingCapture else { return }
        if !recordsScanDiagnostics && minimumCaptureDistance > 0 && (_depthGuidance?.allowsStart != true || ProcessInfo.processInfo.systemUptime - _lastDepthGuidanceTime > 1) {
            _state = .default
            delegate?.scanningViewController?(self, captureGuidance: _depthGuidance?.message ?? "Aim at the whole foot and wait for depth detail.")
            return
        }
        _captureLifetime.invalidate()
        _captureState.begin()
        _prepareDiagnosticsIfNeeded()
        _diagnostics?.beginCapture()
        if usesDeviceMotionGravity && !recordsScanDiagnostics {
            guard _motionManager.isDeviceMotionAvailable, _motionManager.deviceMotion != nil else {
                _failCapture("Motion sensing is not ready. Hold the phone still for a moment, then try again.")
                return
            }
        }
        ScanningHapticFeedbackEngine.shared.scanningBegan()
        _state = .scanning
        _assimilatedFrameIndex = 0
        meshTexturing.reset()
    }

    /// An immutable observed-point snapshot for background geometry checks.
    /// The caller must still validate geometry; elapsed time never establishes completeness.
    @MainActor public func capturePointCloudSnapshot() -> SCPointCloud? {
        guard _state == .scanning, !_isCompletingCapture,
              !_captureState.isRecovering, let gravity = _captureState.gravity else { return nil }
        let cloud = _reconstructionManager.buildPointCloud()
        guard cloud.pointCount > 0 else { return nil }
        return cloud.settingGravity(gravity)
    }

    /** Stops scanning immediately */
    @objc public func stopScanning(reason: ScanningTerminationReason) {
        if _diagnostics != nil {
            _stopDiagnosticCapture(reason: reason == .finished ? "Stopped by recording button" : "Capture canceled",
                                   incomplete: reason == .canceled)
            return
        }
        if reason == .canceled { _captureLifetime.invalidate() }
        guard _state == _State.scanning else { return }
        
        _captureLifetime.invalidate()
        _state = .default
        _latestViewMatrix = matrix_identity_float4x4
        _updateUI()
        
        switch reason {
        case .canceled: ScanningHapticFeedbackEngine.shared.scanningCanceled()
        case .finished: ScanningHapticFeedbackEngine.shared.scanningFinished()
        }
        
        if reason == .finished {
            let completionGeneration = _captureLifetime.generation
            _isCompletingCapture = true
            _cameraManager.stopSession()
            
            meshTexturing.cameraCalibrationData = _reconstructionManager.latestCameraCalibrationData
            meshTexturing.cameraCalibrationFrameWidth = _reconstructionManager.latestCameraCalibrationFrameWidth
            meshTexturing.cameraCalibrationFrameHeight = _reconstructionManager.latestCameraCalibrationFrameHeight
            
            // Do final cleanup on the scan
            _reconstructionManager.finalize {
                guard self._captureLifetime.accepts(completionGeneration) else {
                    self._diagnostics = nil
                    self._reconstructionManager.reset()
                    self._isCompletingCapture = false
                    return
                }
                let rawCloud = self._reconstructionManager.buildPointCloud()
                let pointCloud: SCPointCloud
                if self.usesDeviceMotionGravity, let gravity = self._captureState.gravity {
                    pointCloud = rawCloud.settingGravity(gravity)
                } else if self.usesDeviceMotionGravity {
                    let message = "No stable depth frames were captured. Keep the whole foot and some floor in view, then try again."
                    self._finishDiagnostics(rawCloud.settingGravity(.zero), reason: message) {
                        self._reconstructionManager.reset()
                        self._isCompletingCapture = false
                        guard self._captureLifetime.accepts(completionGeneration) else { return }
                        self.delegate?.scanningViewController?(self, didFail: message)
                    }
                    return
                } else {
                    pointCloud = rawCloud
                }

                self._finishDiagnostics(pointCloud, reason: "Capture finished; geometry not yet validated") {
                    self._reconstructionManager.reset()
                    self._isCompletingCapture = false
                    guard self._captureLifetime.accepts(completionGeneration) else { return }
                    self.delegate?.scanningViewController?(self, didScan: pointCloud)
                }
            }
        } else {
            _reconstructionManager.reset()
            meshTexturing.reset()
        }
    }
    
    @objc public var maxDepthResolution: Int = 320 {
        didSet {
            if isViewLoaded && oldValue != maxDepthResolution {
                _cameraManager.configureCaptureSession(maxResolution: maxDepthResolution)
            }
        }
    }
    
    /** To manually pause the camera output, set this to true */
    @objc public var isCameraPaused: Bool = false {
        didSet {
            guard oldValue != isCameraPaused else { return }
            
            if isCameraPaused {
                _cameraManager.stopSession()
            } else {
                _cameraManager.startSession(nil)
            }
        }
    }
    
    /** If true, displays a button that flips the output horizontally for scanning with a mirror bracket */
    @objc public var showsMirrorModeButton: Bool = false {
        didSet { _updateUI() }
    }
    
    @objc public var mirrorModeEnabled: Bool {
        get { return _mirrorModeButton.isSelected }
        set {
            _mirrorModeButton.isSelected = newValue
            meshTexturing.flipsInputHorizontally = newValue
        }
    }
    
    @objc public var generatesTexturedMeshes: Bool = false {
        didSet { if isViewLoaded { _reconstructionManager.includesColorBuffersInMetadata = generatesTexturedMeshes } }
    }
    @objc public var texturedMeshColorBufferSaveInterval: Int = 8
    @objc public lazy var meshTexturing = SCMeshTexturing()
    
    // MARK: - UIViewController
    open override var preferredStatusBarStyle: UIStatusBarStyle { .lightContent }
    
    override open func viewDidLoad() {
        super.viewDidLoad()
        _prepareDiagnosticsIfNeeded()
    
        _setUpSubviews()
        
        _cameraManager.delegate = self
        _cameraManager.configureCaptureSession(maxResolution: maxDepthResolution)
        
        _reconstructionManager.delegate = self
        _reconstructionManager.includesColorBuffersInMetadata = generatesTexturedMeshes
        _reconstructionManager.cameraFocalLengthScaleFactor = cameraFocalLengthScaleFactor
        if reconstructionIterationLimit > 0 { _reconstructionManager.maxICPIterations = reconstructionIterationLimit }
        if maximumReconstructionDepth > 0 {
            _reconstructionManager.maxDepth = maximumReconstructionDepth
        }
        
        NotificationCenter.default.addObserver(self, selector: #selector(_thermalStateChanged), name: ProcessInfo.thermalStateDidChangeNotification, object: nil)
    }
    
    override open func viewDidAppear(_ animated: Bool) {
        super.viewDidAppear(animated)
        
        guard CameraManager.isDepthCameraAvailable else { return }
        
        if usesDeviceMotionGravity {
            _motionManager.deviceMotionUpdateInterval = 1.0 / 60.0
            _motionManager.startDeviceMotionUpdates()
        }
        _startCameraSession()
    }
    
    override open func viewWillDisappear(_ animated: Bool) {
        super.viewWillDisappear(animated)
        
        _captureLifetime.invalidate()
        stopScanning(reason: ScanningViewController.ScanningTerminationReason.canceled)
        
        _cameraManager.stopSession()
        _motionManager.stopDeviceMotionUpdates()
    }
    
    override open func viewDidLayoutSubviews() {
        super.viewDidLayoutSubviews()
        
        _metalContainerView.frame = view.bounds
        
        CATransaction.begin()
        CATransaction.disableActions()
        _metalLayer.frame = _metalContainerView.bounds
        _metalLayer.drawableSize = CGSize(width:  _metalLayer.frame.width  * _metalLayer.contentsScale,
                                          height: _metalLayer.frame.height * _metalLayer.contentsScale)
        CATransaction.commit()
        
        _countdownLabel.sizeToFit()
        _countdownLabel.center = _metalContainerView.center
        _scanFailedLabel.sizeToFit()
        _scanFailedLabel.center = _metalContainerView.center
        dismissButton.sizeToFit()
        dismissButton.center = CGPoint(x: 20 + 0.5 * dismissButton.frame.width,
                                       y: 0.5 * dismissButton.frame.height + view.safeAreaInsets.top)
        
        _mirrorModeBackground.frame = CGRect(x: 0, y: 0, width: view.bounds.width, height: dismissButton.frame.maxY + 15)
        _mirrorModeLabel.sizeToFit()
        _mirrorModeLabel.center = CGPoint(x: view.bounds.midX,
                                          y: dismissButton.center.y)
        _mirrorModeButton.sizeToFit()
        _mirrorModeButton.center = CGPoint(x: view.bounds.maxX - 0.5 * _mirrorModeButton.frame.width - 20,
                                           y: dismissButton.center.y)
        
        shutterButton.sizeToFit()
        shutterButton.center = CGPoint(x: view.bounds.midX,
                                       y: view.bounds.maxY - 5 - 0.5 * shutterButton.frame.size.height - view.safeAreaInsets.bottom)
    }
    
    override open func didReceiveMemoryWarning() {
        print("Received low memory warning; stopping scanning")
        if usesDeviceMotionGravity {
            _failCapture("The scan stopped because memory is low. Close other apps and try a shorter scan.")
        } else { stopScanning(reason: .finished) }
    }
    
    // MARK: - Notifications
    
    @objc private func _focusOnTap(_ gesture: UITapGestureRecognizer) {
        // Disallow this while scanning
        guard _state != _State.scanning else { return }
        
        let location = gesture.location(in: view)
        
        _cameraManager.focusOnTap(at: location)
    }
    
    @objc private func _thermalStateChanged(notification: Notification) {
        guard let processInfo = notification.object as? ProcessInfo,
            processInfo.thermalState == .critical
            else { return }
        
        DispatchQueue.main.async(execute: _stopScanningForCriticalThermalState)
    }
    
    // MARK: - CameraManagerDelegate
    
    public func cameraDidOutput(colorBuffer: CVPixelBuffer, depthBuffer: CVPixelBuffer, depthCalibrationData: AVCameraCalibrationData) {
        cameraDidOutput(colorBuffer: colorBuffer, depthBuffer: depthBuffer, depthCalibrationData: depthCalibrationData,
                        colorTimestamp: .invalid, depthTimestamp: .invalid)
    }

    public func cameraManager(_ manager: CameraManager, didDropColorAt colorTimestamp: CMTime,
                              depthAt depthTimestamp: CMTime, colorDropped: Bool, depthDropped: Bool) {
        var recorder: ScanDiagnosticsRecorder?
        DispatchQueue.main.sync { if self._state == .scanning { recorder = self._diagnostics } }
        recorder?.recordCameraDrop(colorTimestamp: colorTimestamp, depthTimestamp: depthTimestamp,
                                   colorDropped: colorDropped, depthDropped: depthDropped)
    }

    public func cameraDidOutput(colorBuffer: CVPixelBuffer, depthBuffer: CVPixelBuffer,
                               depthCalibrationData: AVCameraCalibrationData,
                               colorTimestamp: CMTime, depthTimestamp: CMTime) {
        var isScanning = false
        var shouldAccumulate = false
        var generation: UInt = 0
        var viewMatrix = matrix_identity_float4x4
        var recorder: ScanDiagnosticsRecorder?
        var motion: CMDeviceMotion?
        var withheldReason = "not_scanning"
        DispatchQueue.main.sync {
            isScanning = self._state == .scanning
            generation = self._captureLifetime.generation
            if isScanning { recorder = self._diagnostics; motion = self._motionManager.deviceMotion }
            viewMatrix = self._latestViewMatrix
            let now = ProcessInfo.processInfo.systemUptime
            if self.minimumCaptureDistance > 0 {
                self._updateDepthGuidance(depthBuffer)
                self._lastDepthGuidanceTime = now
            }
            shouldAccumulate = isScanning
            if isScanning {
                // Preserve raw evidence even when a depth frame is withheld from fusion.
                if self.minimumCaptureDistance > 0 {
                    let wasRecovering = self._captureState.isRecovering
                    let usable = self._depthGuidance?.allowsStart == true
                    let expired = self._captureState.updateDepthUsable(usable, now: now, grace: 8)
                    shouldAccumulate = usable
                    if !usable && !wasRecovering {
                        self.delegate?.scanningViewController?(self, trackingIsStable: false)
                    }
                    if expired && !self.recordsScanDiagnostics {
                        self._failCapture("Depth detail did not recover. Keep the whole foot and some floor visible at about 30–40 cm, then try again.")
                        shouldAccumulate = false
                    }
                    self._publishCaptureProgress()
                }
            }
            // Freeze gravity at the first depth frame actually submitted to fusion.
            if shouldAccumulate && self.usesDeviceMotionGravity && self._captureState.gravity == nil {
                if let motion = self._motionManager.deviceMotion {
                    let g = motion.gravity
                    shouldAccumulate = self._captureState.recordInitialGravity(SIMD3(Float(g.x), Float(g.y), Float(g.z)))
                } else {
                    shouldAccumulate = false
                }
            }
            withheldReason = shouldAccumulate ? "requested" : (isScanning ? "depth_or_motion_withheld" : "not_scanning")
        }

        let diagnosticSequence = recorder?.recordFrame(color: colorBuffer, depth: depthBuffer,
            calibration: depthCalibrationData, colorTimestamp: colorTimestamp, depthTimestamp: depthTimestamp,
            motion: motion, submission: ["submitted": false, "requested": shouldAccumulate,
                "reason": withheldReason, "captureGeneration": generation])
        if recorder != nil && diagnosticSequence == nil { shouldAccumulate = false }

        let pointCloud: SCPointCloud
        
        if isScanning {
            pointCloud = _reconstructionManager.buildPointCloud()
        } else {
            // When the user is not scanning, render a preview by reconstructing the most recent depth buffer
            // into a point cloud from the current point of view, drawn on top of the RGB camera
            // As the result is never saved and the RGB color not used for visualization, there is no need to
            // pass it the color buffer to build the point cloud
            pointCloud = _reconstructionManager.reconstructSingleDepthBuffer(depthBuffer,
                                                                             colorBuffer: nil,
                                                                             with: depthCalibrationData,
                                                                             smoothingPoints: true)
        }
        
        scanningViewRenderer.draw(colorBuffer: colorBuffer,
                                  pointCloud: pointCloud,
                                  depthCameraCalibrationData: depthCalibrationData,
                                  viewMatrix: viewMatrix,
                                  into: _metalLayer)
        
        if shouldAccumulate {
            // Rendering can outlive a stop/finalize. Never enqueue that stale camera frame.
            DispatchQueue.main.sync {
                guard self._state == .scanning, !self._isCompletingCapture,
                      self._captureLifetime.accepts(generation) else { return }
                if let recorder, let diagnosticSequence {
                    recorder.recordEvent("fusion_submitted", fields: ["inputSequence": diagnosticSequence])
                    self._reconstructionManager.accumulate(depthBuffer: depthBuffer, colorBuffer: colorBuffer,
                        calibrationData: depthCalibrationData, diagnosticSequence: diagnosticSequence)
                } else {
                    self._reconstructionManager.accumulate(depthBuffer: depthBuffer, colorBuffer: colorBuffer,
                        calibrationData: depthCalibrationData)
                }
            }
        }
    }
    
    private func _updateDepthGuidance(_ buffer: CVPixelBuffer) {
        guard CVPixelBufferGetPixelFormatType(buffer) == kCVPixelFormatType_DepthFloat32 else { return }
        CVPixelBufferLockBaseAddress(buffer, .readOnly)
        defer { CVPixelBufferUnlockBaseAddress(buffer, .readOnly) }
        guard let base = CVPixelBufferGetBaseAddress(buffer) else { return }
        let width = CVPixelBufferGetWidth(buffer), height = CVPixelBufferGetHeight(buffer)
        let rowBytes = CVPixelBufferGetBytesPerRow(buffer)
        var samples: [Float] = []
        for y in stride(from: height / 6, to: height * 5 / 6, by: 6) {
            let row = base.advanced(by: y * rowBytes).assumingMemoryBound(to: Float.self)
            for x in stride(from: width / 6, to: width * 5 / 6, by: 6) { samples.append(row[x]) }
        }
        let guidance = ScanDepthGuidance(samples: samples, minimumDistance: minimumCaptureDistance, maximumDistance: maximumCaptureDistance)
        let now = ProcessInfo.processInfo.systemUptime
        if _lastReportedDepthMessage != guidance.message && now - _lastDepthMessageTime >= 0.25 {
            _lastReportedDepthMessage = guidance.message
            _lastDepthMessageTime = now
            delegate?.scanningViewController?(self, captureGuidance: guidance.message)
        }
        _depthGuidance = guidance
    }

    // MARK: - SCReconstructionManagerDelegate
    
    public func reconstructionManager(_ manager: SCReconstructionManager, didProcessWith metadata: SCAssimilatedFrameMetadata, statistics: SCReconstructionManagerStatistics) {
        guard _state == .scanning else { return }
        
        _diagnostics?.recordTracking(metadata, statistics: statistics)
        // Late metadata from a frame queued before a depth pause cannot clear that pause.
        guard !_captureState.isDepthPaused else { _publishCaptureProgress(); return }
        switch metadata.result {
        case .succeeded, .poorTracking:
            _latestViewMatrix = metadata.viewMatrix
            if _captureState.isRecovering {
                delegate?.scanningViewController?(self, trackingIsStable: true)
            }
            _captureState.trackingRecovered()
            _captureState.recordAccepted(now: ProcessInfo.processInfo.systemUptime, viewMatrix: metadata.viewMatrix)
            _publishCaptureProgress()
            // Save off every nth frame
            if
                generatesTexturedMeshes
                && _assimilatedFrameIndex % texturedMeshColorBufferSaveInterval == 0,
                let colorBuffer = metadata.colorBuffer?.takeUnretainedValue()
            {
                meshTexturing.saveColorBufferForReconstruction(colorBuffer,
                                                               withViewMatrix: metadata.viewMatrix,
                                                               projectionMatrix: metadata.projectionMatrix)
            }
            _assimilatedFrameIndex += 1
            
        case .failed, .lostTracking:
            if trackingRecoveryTime > 0 {
                if !_captureState.isRecovering {
                    delegate?.scanningViewController?(self, trackingIsStable: false)
                }
                if _captureState.shouldStopForTrackingLoss(
                    now: ProcessInfo.processInfo.systemUptime, grace: trackingRecoveryTime
                ) {
                    if !recordsScanDiagnostics {
                        _failCapture("Tracking was lost. Return to the last angle, move more slowly and keep the foot still. Start with the whole foot and some floor visible.")
                    }
                }
            } else if metadata.result == .failed {
                let assimilatedTooFewFrames = statistics.succeededCount < _failedScanShowPreviewMinFrameCount
                stopScanning(reason: assimilatedTooFewFrames ? .canceled : .finished)
            }
        @unknown default:
            break
        }
        _publishCaptureProgress()
    }
    
    public func reconstructionManager(_ manager: SCReconstructionManager, didEncounterAPIError error: Error) {
        print("SCReconstructionManager hit API error: \(error)")
        if let recorder = _diagnostics {
            recorder.recordEvent("fusion_api_error", fields: ["message": error.localizedDescription])
            return
        }
        _failCapture("The depth camera was interrupted. Close the scanner and try again.")
    }

    public func cameraManager(_ manager: CameraManager, didInterrupt message: String) {
        if _state == .scanning || _diagnostics != nil { _failCapture(message) }
    }

    private func _publishCaptureProgress() {
        let guidance = _captureState.isDepthPaused ? _depthGuidance?.message :
            (_captureState.isRecovering ? "Return to the last angle and move slowly." : nil)
        onCaptureProgress?(_captureState.progress(guidance: guidance))
    }

    private func _prepareDiagnosticsIfNeeded() {
        guard recordsScanDiagnostics, _diagnostics == nil else { return }
        let recorder = ScanDiagnosticsRecorder(configuration: [
            "maximumDepthMetres": maximumReconstructionDepth, "icpIterationLimit": reconstructionIterationLimit,
            "focalLengthScaleFactor": cameraFocalLengthScaleFactor, "minimumCaptureDistance": minimumCaptureDistance,
            "maximumCaptureDistance": maximumCaptureDistance, "usesDeviceMotionGravity": usesDeviceMotionGravity,
            "maxDepthResolution": maxDepthResolution, "trackingRecoveryTime": trackingRecoveryTime,
            "flipsInputHorizontally": mirrorModeEnabled, "normalizedFrameClipRegion": [0,0,0,0]
        ]) { [weak self] message in self?._failCapture(message) }
        _diagnostics = recorder
        _reconstructionManager.diagnosticEventHandler = { [weak recorder] event in
            guard let kind = event["event"] as? String else { return }
            recorder?.recordEvent(kind, fields: event as? [String: Any] ?? [:])
        }
    }

    private func _stopDiagnosticCapture(reason: String, incomplete: Bool) {
        guard !_isCompletingCapture, let recorder = _diagnostics else { return }
        _isCompletingCapture = true
        _captureLifetime.invalidate()
        _state = .default
        _cameraManager.stopSession { [self] in
            // Session stop drains camera callbacks. Seal the captured RGBD prefix
            // even if fusion is still busy, tracking failed or no points exist.
            _reconstructionManager.reset()
            recorder.finish(reason: reason, gravity: _captureState.gravity, incomplete: incomplete) { [self] url in
                _diagnostics = nil
                _reconstructionManager.diagnosticEventHandler = nil
                if let url { delegate?.scanningViewController?(self, didWriteDiagnostics: url) }
                else { delegate?.scanningViewController?(self, didFail: "The private recording could not be saved.") }
            }
        }
    }

    private func _finishDiagnostics(_ cloud: SCPointCloud, reason: String, completion: @escaping () -> Void) {
        guard let recorder = _diagnostics else { completion(); return }
        _diagnostics = nil
        let diagnosticGeneration = _captureLifetime.generation
        recorder.finish(reason: reason, gravity: cloud.gravity) { url in
            if self._captureLifetime.accepts(diagnosticGeneration), let url = url {
                self.delegate?.scanningViewController?(self, didWriteDiagnostics: url)
            }
            completion()
        }
    }

    private func _failCapture(_ message: String) {
        if _diagnostics != nil {
            _stopDiagnosticCapture(reason: message, incomplete: true)
            return
        }
        guard !_isCompletingCapture else { return }
        guard _diagnostics != nil else {
            stopScanning(reason: .canceled)
            if delegate?.scanningViewController?(self, didFail: message) == nil { _showScanFailedMessage() }
            return
        }
        // Snapshot only after the model queue is quiescent; reset would destroy the evidence.
        _isCompletingCapture = true
        _captureLifetime.invalidate()
        let completionGeneration = _captureLifetime.generation
        _state = .default
        _updateUI()
        _cameraManager.stopSession()
        _reconstructionManager.finalize {
            guard self._captureLifetime.accepts(completionGeneration) else {
                self._diagnostics = nil
                self._reconstructionManager.reset()
                self._isCompletingCapture = false
                return
            }
            let cloud = self._reconstructionManager.buildPointCloud().settingGravity(self._captureState.gravity ?? .zero)
            self._finishDiagnostics(cloud, reason: message) {
                self._reconstructionManager.reset()
                self.meshTexturing.reset()
                self._isCompletingCapture = false
                guard self._captureLifetime.accepts(completionGeneration) else { return }
                if self.delegate?.scanningViewController?(self, didFail: message) == nil { self._showScanFailedMessage() }
            }
        }
    }

    // MARK: - Private properties
        
    private let _metalDevice = MTLCreateSystemDefaultDevice()!
    private lazy var _algorithmCommandQueue = _metalDevice.makeCommandQueue()!
    private lazy var _visualizationCommandQueue = _metalDevice.makeCommandQueue()!
    private lazy var _reconstructionManager = SCReconstructionManager(device: _metalDevice, commandQueue: _algorithmCommandQueue, maxThreadCount: _maxReconstructionThreadCount)
    private let _cameraManager = CameraManager()
    private var _latestViewMatrix = matrix_identity_float4x4
    private var _assimilatedFrameIndex = 0
    
    private let _metalContainerView = UIView()
    private let _metalLayer = CAMetalLayer()
    private let _countdownLabel = UILabel()
    private let _scanFailedLabel = UILabel()
    
    private let _mirrorModeBackground = UIView()
    private let _mirrorModeLabel = UILabel()
    private let _mirrorModeButton = UIButton()
    
    // MARK: - UI State Management
    
    private enum _State: Equatable {
        case `default`
        case countdownSeconds(Int)
        case scanning
    }
    
    private var _state = _State.default {
        didSet {
            _updateUI()
            
            // Prevent auto screen dimming/lock while scanning
            UIApplication.shared.isIdleTimerDisabled = _state == _State.scanning
        }
    }
    
    private func _setUpSubviews() {
        view.backgroundColor = UIColor.black
        
        _metalLayer.isOpaque = true
        _metalLayer.contentsScale = UIScreen.main.scale
        _metalLayer.device = _metalDevice
        _metalLayer.pixelFormat = MTLPixelFormat.bgra8Unorm
        _metalLayer.framebufferOnly = false
        
        _metalContainerView.layer.addSublayer(_metalLayer)
        view.addSubview(_metalContainerView)
        view.addSubview(_countdownLabel)
        view.addSubview(_scanFailedLabel)
        view.addSubview(_mirrorModeBackground)
        view.addSubview(dismissButton)
        view.addSubview(shutterButton)
        _mirrorModeBackground.addSubview(_mirrorModeLabel)
        _mirrorModeBackground.addSubview(_mirrorModeButton)
        
        let mirrorModeText = NSMutableAttributedString(string: "Mirror Mode On\n", attributes: [.font: UIFont.systemFont(ofSize: 12, weight: .bold)])
        mirrorModeText.append(NSAttributedString(string: "Attach Mirror Clip", attributes: [.font: UIFont.systemFont(ofSize: 12, weight: .regular)]))
        
        _mirrorModeBackground.backgroundColor = UIColor(white: 0, alpha: 0.28)
        _mirrorModeLabel.attributedText = mirrorModeText
        _mirrorModeLabel.textColor = UIColor.white
        _mirrorModeLabel.textAlignment = NSTextAlignment.center
        _mirrorModeLabel.numberOfLines = 2
        _mirrorModeButton.addTarget(self, action: #selector(toggleMirrorMode(_:)), for: UIControl.Event.touchUpInside)
        _mirrorModeButton.setImage(UIImage(named: "FlipCamera", in: Bundle.scuiResourcesBundle, compatibleWith: nil)!, for: UIControl.State.normal)
        
        _countdownLabel.textColor = UIColor.white
        _countdownLabel.textAlignment = NSTextAlignment.center
        _countdownLabel.font = UIFont.systemFont(ofSize: 96, weight: UIFont.Weight.semibold)
        
        _scanFailedLabel.text = "Scan failed!\nMove the device slowly\nand keep the subject still"
        _scanFailedLabel.numberOfLines = 0
        _scanFailedLabel.textAlignment = NSTextAlignment.center
        _scanFailedLabel.font = UIFont.systemFont(ofSize: 24, weight: UIFont.Weight.medium)
        _scanFailedLabel.backgroundColor = UIColor(white: 1.0, alpha: 0.8)
        _scanFailedLabel.isHidden = true
        
        dismissButton.setImage(UIImage(named: "Dismiss", in: Bundle.scuiResourcesBundle, compatibleWith: nil), for: UIControl.State.normal)
        dismissButton.addTarget(self, action: #selector(dismissTapped(_:)), for: UIControl.Event.touchUpInside)
        shutterButton.addTarget(self, action: #selector(shutterTapped(_:)), for: UIControl.Event.touchUpInside)
        
        view.addGestureRecognizer(UITapGestureRecognizer(target: self, action: #selector(_focusOnTap)))
    }
    
    private func _updateUI() {
        loadViewIfNeeded()
        
        switch _state {
        case .default:
            shutterButton.shutterButtonState = .default
            
        case .countdownSeconds(let seconds):
            shutterButton.shutterButtonState = .countdown
            _countdownLabel.isHidden = seconds == 0
            _countdownLabel.text = "\(seconds)"
            _countdownLabel.sizeToFit()
            
        case .scanning:
            shutterButton.shutterButtonState = .scanning
            
        }
        
        _cameraManager.isFocusLocked = _state == .scanning
        
        _mirrorModeBackground.isHidden = !showsMirrorModeButton
        _mirrorModeLabel.isHidden = !mirrorModeEnabled
        scanningViewRenderer.flipsInputHorizontally = mirrorModeEnabled
        _reconstructionManager.flipsInputHorizontally = mirrorModeEnabled
    }
    
    private func _startCameraSession() {
        _cameraManager.startSession { result in
            switch result {
            case .success:
                break
            case .configurationFailed:
                print("Configuration failed for an unknown reason")
                if self._diagnostics != nil { self._stopDiagnosticCapture(reason: "Camera configuration failed before capture", incomplete: true) }
            case .notAuthorized:
                let message = "To take a 3D scan, go to your privacy settings. Tap Camera and turn on for Capture"
                let alertController = UIAlertController(title: "Camera Access", message: message, preferredStyle: .alert)
                alertController.addAction(UIAlertAction(title: "OK",
                                                        style: .cancel,
                                                        handler: nil))
                alertController.addAction(UIAlertAction(title: "Open Settings",
                                                        style: .`default`)
                { _ in
                    UIApplication.shared.open(URL.init(string: UIApplication.openSettingsURLString)!, options: [:], completionHandler: nil)
                })
                
                self.present(alertController, animated: true)
            }
        }
    }
    
    private func _startCountdown(_ completion: @escaping () -> Void) {
        guard countdownStartCount > 0 else {
            completion()
            return
        }
        
        _state = .countdownSeconds(countdownStartCount)
        _iterateCountdown(completion)
    }
    
    private func _cancelCountdown() {
        ScanningHapticFeedbackEngine.shared.scanningCanceled()
        _countdownLabel.alpha = 0
        _state = .default
    }
    
    private func _iterateCountdown(_ completion: @escaping () -> Void) {
        ScanningHapticFeedbackEngine.shared.countdownCountedDown()
        
        if case let _State.countdownSeconds(seconds) = _state, seconds == 0 {
            completion()
            return
        }
        
        _countdownLabel.alpha = 1
        UIView.animate(withDuration: countdownPerSecondDuration, animations: {
            self._countdownLabel.alpha = 0
        }) { finished in
            if
                finished,
                case let _State.countdownSeconds(seconds) = self._state,
                seconds > 0
            {
                self._state = _State.countdownSeconds(seconds - 1)
                self._iterateCountdown(completion)
            }
        }
    }
    
    private let _failedScanShowPreviewMinFrameCount = 50
    
    private lazy var _maxReconstructionThreadCount: Int32 = {
        // Unfortunately, there's not a good way to get the number of *high-performance*
        // CPU cores on iOS, so we have to hard-code this for now
        return UIDevice.current.userInterfaceIdiom == .pad ? 4 : 2
    }()
    
    private func _showScanFailedMessage() {
        _scanFailedLabel.isHidden = false
        _scanFailedLabel.alpha = 1
        
        UIView.animate(withDuration: 0.8, delay: 3.0, options: [], animations: {
            self._scanFailedLabel.alpha = 0
        }, completion: { finished in
            self._scanFailedLabel.isHidden = true
        })
    }
    
    private func _stopScanningForCriticalThermalState() {
        if usesDeviceMotionGravity && _state == .scanning {
            _failCapture("The phone is too warm to continue scanning. Let it cool down, then try again.")
            return
        }
        if _state == _State.scanning {
            self.stopScanning(reason: .finished)
        }
        
        let deviceName = UIDevice.current.userInterfaceIdiom == .pad ? "iPad" : "iPhone"
        let alertController = UIAlertController(title: "\(deviceName) is too hot!",
            message: "Please allow \(deviceName) to cool down and try again",
            preferredStyle: .alert)
        alertController.addAction(UIAlertAction(title: "OK", style: .cancel, handler: nil))
        self.present(alertController, animated: true)
    }
    
    @objc private func toggleMirrorMode(_ sender: UIButton) {
        sender.isSelected = !sender.isSelected
        _updateUI()
    }
    
    @objc private func dismissTapped(_ sender: UIButton?) {
        delegate?.scanningViewControllerDidCancel(self)
    }
    
}
