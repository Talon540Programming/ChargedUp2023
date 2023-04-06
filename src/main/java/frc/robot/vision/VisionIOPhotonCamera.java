package frc.robot.vision;

import edu.wpi.first.math.geometry.Transform3d;
import frc.lib.vision.EstimatedRobotPose;
import frc.lib.vision.VisionPoseEstimator;
import java.util.ArrayList;
import java.util.List;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionIOPhotonCamera implements VisionIO {
  private final PhotonCamera m_camera;

  private final VisionPoseEstimator m_poseEstimator;

  public VisionIOPhotonCamera(String cameraName, Transform3d robotToCamera) {
    m_camera = new PhotonCamera(cameraName);
    m_poseEstimator = new VisionPoseEstimator(m_camera, robotToCamera);
  }

  @Override
  public void updateInputs(VisionIOInputs inputs) {
    PhotonPipelineResult result = m_camera.getLatestResult();

    inputs.Connected = m_camera.isConnected();
    inputs.PipelineIndex = m_camera.getPipelineIndex();
    inputs.DriverMode = m_camera.getDriverMode();
    inputs.LEDMode = m_camera.getLEDMode().value;

    inputs.CaptureTimestampSeconds = result.getTimestampSeconds();
    inputs.LatencyMilliseconds = result.getLatencyMillis();
    inputs.HasTargets = result.hasTargets();

    // Log the corners of targets for visualization
    List<Double> cornerXList = new ArrayList<>();
    List<Double> cornerYList = new ArrayList<>();

    for (PhotonTrackedTarget target : result.getTargets()) {
      for (TargetCorner corner : target.getDetectedCorners()) {
        cornerXList.add(corner.x);
        cornerYList.add(corner.y);
      }
    }

    inputs.CornerX = cornerXList.stream().mapToDouble(Double::doubleValue).toArray();
    inputs.CornerY = cornerYList.stream().mapToDouble(Double::doubleValue).toArray();
  }

  @Override
  public Optional<EstimatedRobotPose> getEstimatedPose() {
    return m_poseEstimator.getRobotPose();
  }

  @Override
  public String getCameraName() {
    return m_camera.getName();
  }
}
