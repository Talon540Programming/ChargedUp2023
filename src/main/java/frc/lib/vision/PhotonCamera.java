package frc.lib.vision;

import edu.wpi.first.math.geometry.Transform3d;

import java.util.Objects;
import java.util.function.Supplier;
import org.littletonrobotics.junction.LogTable;
import org.littletonrobotics.junction.inputs.LoggableInputs;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

public class PhotonCamera extends org.photonvision.PhotonCamera implements LoggableInputs {
  private final Supplier<Transform3d> kRobotToCamera;

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

  public TrackingMode getTrackingMode() {
    final int pipelineIndex = getPipelineIndex();
    for (TrackingMode val : TrackingMode.values()) {
      if (val.pipeline == pipelineIndex) return val;
    }

    return TrackingMode.kUnknown;
  }

  public void setTrackingMode(TrackingMode mode) {
    setPipelineIndex(mode.pipeline);
  }

  @Override
  public void toLog(LogTable table) {
    PhotonPipelineResult result = getLatestResult();

    table.put("Timestamp", result.getTimestampSeconds());
    table.put("Latency", result.getLatencyMillis());
    table.put("HasTargets", result.hasTargets());

    for (int i = 0; i < result.targets.size() - 1; i++) {
      PhotonTrackedTarget target = result.targets.get(i);
      table.put("Targets/" + i + "/Yaw", target.getYaw());
      table.put("Targets/" + i + "/Pitch", target.getPitch());
      table.put("Targets/" + i + "/Area", target.getArea());
      table.put("Targets/" + i + "/Skew", target.getSkew());
      table.put("Targets/" + i + "/FiducialId", target.getFiducialId());
      table.put("Targets/" + i + "/BestCameraToTarget", target.getBestCameraToTarget().toString());
      table.put(
          "Targets/" + i + "/AltCameraToTarget", target.getAlternateCameraToTarget().toString());
      table.put("Targets/" + i + "/PoseAmbiguity", target.getPoseAmbiguity());
    }
  }

  @Override
  public void fromLog(LogTable table) {
    // TODO, populate data from AdvantageKit
  }
}
