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
  // Set default tracking mode to AprilTag
  public PhotonCamera(String subtableName, Supplier<Transform3d> robotToCameraSupplier) {
    super(subtableName);
    kRobotToCamera = Objects.requireNonNull(robotToCameraSupplier);
  }

  public PhotonCamera(String subtableName, Transform3d robotToCamera) {
    this(subtableName, () -> robotToCamera);
  }

  public Transform3d getRobotToCamera() {
    return kRobotToCamera.get();
  }

  public void enableLEDs() {
    this.setLED(VisionLEDMode.kOn);
  }

  public void disableLEDs() {
    this.setLED(VisionLEDMode.kOff);
  }

  public void blinkLEDs() {
    this.setLED(VisionLEDMode.kBlink);
  }

  public TargetMode getTrackingMode() {
    final int pipelineIndex = getPipelineIndex();
    for (TargetMode val : TargetMode.values()) {
      if (val.pipeline == pipelineIndex) return val;
    }

    return TargetMode.kUnknown;
  }

  public void setTrackingMode(TargetMode mode) {
    if(mode == TargetMode.kUnknown) {
      return;
    }

    setPipelineIndex(mode.pipeline);

    // Only enable LEDs if the tracking mode is Reflective
    if(mode == TargetMode.kReflective) {
      enableLEDs();
    } else {
      disableLEDs();
    }
  }

  public void logData() {
    String cameraLogPath = "Vision/Cameras/"+getName();
    Logger instance = Logger.getInstance();

    PhotonPipelineResult result = getLatestResult();

    instance.recordOutput(cameraLogPath+"/TargetMode", getTrackingMode().toString());
    instance.recordOutput(cameraLogPath+"/IsDriverMode", getDriverMode());
    instance.recordOutput(cameraLogPath+"/LED_MODE", getLEDMode().toString());

    instance.recordOutput(cameraLogPath+"/CaptureTimestamp", result.getTimestampSeconds());
    instance.recordOutput(cameraLogPath+"/Latency", result.getLatencyMillis());
    instance.recordOutput(cameraLogPath+"/HasTargets", result.hasTargets());

    // Log the corners of targets for visualization
    List<Double> cornerXList = new ArrayList<>();
    List<Double> cornerYList = new ArrayList<>();

    for (PhotonTrackedTarget target : result.getTargets()) {
      for (TargetCorner corner : target.getDetectedCorners()) {
        cornerXList.add(corner.x);
        cornerYList.add(corner.y);
      }
    }

    instance.recordOutput(cameraLogPath+"/CornerX", cornerXList.stream().mapToDouble(Double::doubleValue).toArray());
    instance.recordOutput(cameraLogPath+"/CornerY",cornerYList.stream().mapToDouble(Double::doubleValue).toArray());
  }
}
