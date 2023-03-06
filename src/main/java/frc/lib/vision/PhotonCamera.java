package frc.lib.vision;

import edu.wpi.first.math.geometry.Transform3d;
import java.util.ArrayList;
import java.util.List;
import java.util.Objects;
import java.util.function.Supplier;
import org.littletonrobotics.junction.Logger;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class PhotonCamera extends org.photonvision.PhotonCamera {
  private final Supplier<Transform3d> kRobotToCamera;

  /**
   * Create a PhotonCamera given its position and name.
   *
   * @param cameraSubtableName name of the camera.
   * @param robotToCameraSupplier supplier of the camera's position.
   */
  public PhotonCamera(String cameraSubtableName, Supplier<Transform3d> robotToCameraSupplier) {
    super(cameraSubtableName);
    kRobotToCamera = Objects.requireNonNull(robotToCameraSupplier);
  }

  /**
   * Create a PhotonCamera given its position and name.
   *
   * @param cameraSubtableName name of the camera.
   * @param robotToCamera camera's position as a {@link Transform3d} object.
   */
  public PhotonCamera(String cameraSubtableName, Transform3d robotToCamera) {
    this(cameraSubtableName, () -> robotToCamera);
  }

  /**
   * Get the transformation between the robot origin and camera.
   *
   * @return {@link Transform3d} transform between the robot origin and camera.
   */
  public Transform3d getRobotToCamera() {
    return kRobotToCamera.get();
  }

  /**
   * Enable the LEDs of the camera.
   */
  public void enableLEDs() {
    this.setLED(VisionLEDMode.kOn);
  }

  /**
   * Disable the LEDs of the camera.
   */
  public void disableLEDs() {
    this.setLED(VisionLEDMode.kOff);
  }

  /**
   * Blink the LEDs of the camera.
   */
  public void blinkLEDs() {
    this.setLED(VisionLEDMode.kBlink);
  }

  /**
   * Get the TargetMode of the camera. Will return Unknown if a different pipeline is set.
   *
   * @return currently set camera mode.
   */
  public TargetMode getTargetMode() {
    final int pipelineIndex = getPipelineIndex();
    for (TargetMode val : TargetMode.values()) {
      if (val.pipeline == pipelineIndex) return val;
    }

    return TargetMode.kUnknown;
  }

  /**
   * Set the TrackingMode of the camera.
   *
   * @param mode TrackingMode of the camera.
   */
  public void setTrackingMode(TargetMode mode) {
    if (mode == TargetMode.kUnknown) {
      return;
    }

    setPipelineIndex(mode.pipeline);

    // Only enable LEDs if the tracking mode is Reflective
    if (mode == TargetMode.kReflective) {
      enableLEDs();
    } else {
      disableLEDs();
    }
  }

  /**
   * Log the current state of the camera with the logger.
   * This method should be called periodically in order to correctly log all data.
   */
  public void logData() {
    String cameraLogPath = "Vision/Cameras/" + getName();
    Logger instance = Logger.getInstance();

    PhotonPipelineResult result = getLatestResult();

    instance.recordOutput(cameraLogPath + "/TargetMode", getTargetMode().toString());
    instance.recordOutput(cameraLogPath + "/IsDriverMode", getDriverMode());
    instance.recordOutput(cameraLogPath + "/LED_MODE", getLEDMode().toString());

    instance.recordOutput(cameraLogPath + "/CaptureTimestamp", result.getTimestampSeconds());
    instance.recordOutput(cameraLogPath + "/Latency", result.getLatencyMillis());
    instance.recordOutput(cameraLogPath + "/HasTargets", result.hasTargets());

    // Log the corners of targets for visualization
    List<Double> cornerXList = new ArrayList<>();
    List<Double> cornerYList = new ArrayList<>();

    for (PhotonTrackedTarget target : result.getTargets()) {
      for (TargetCorner corner : target.getDetectedCorners()) {
        cornerXList.add(corner.x);
        cornerYList.add(corner.y);
      }
    }

    instance.recordOutput(
        cameraLogPath + "/CornerX",
        cornerXList.stream().mapToDouble(Double::doubleValue).toArray());
    instance.recordOutput(
        cameraLogPath + "/CornerY",
        cornerYList.stream().mapToDouble(Double::doubleValue).toArray());
  }
}
