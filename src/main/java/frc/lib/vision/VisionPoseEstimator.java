package frc.lib.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.numbers.*;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.ArrayList;
import java.util.Optional;
import org.photonvision.PhotonCamera;
import org.photonvision.estimation.PNPResults;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

public class VisionPoseEstimator {
  private final PhotonCamera m_camera;
  private final Transform3d m_robotToCamera;
  private final AprilTagFieldLayout m_fieldLayout;

  private double m_timestampCacheSeconds = -1.0;

  public VisionPoseEstimator(
      PhotonCamera camera, Transform3d robotToCamera, AprilTagFieldLayout fieldLayout) {
    m_camera = camera;
    m_robotToCamera = robotToCamera;
    m_fieldLayout = fieldLayout;
  }

  public Optional<EstimatedRobotPose> getRobotPose() {
    PhotonPipelineResult pipelineResult = m_camera.getLatestResult();
    double pipelineTimestamp = pipelineResult.getTimestampSeconds();

    if (pipelineTimestamp < 0) {
      return Optional.empty();
    }

    if (0 < m_timestampCacheSeconds
        && Math.abs(m_timestampCacheSeconds - pipelineTimestamp) < 1e-6) {
      return Optional.empty();
    }

    m_timestampCacheSeconds = pipelineTimestamp;

    if (!pipelineResult.hasTargets()) {
      return Optional.empty();
    }

    if (pipelineResult.getTargets().size() < 2) {
      // Use the lowest ambiguity target because there aren't enough targets for MultiTag PNP
      PhotonTrackedTarget lowestAmbiguityTarget = null;
      double lowestAmbiguityScore = 10;

      for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
        double targetPoseAmbiguity = target.getPoseAmbiguity();
        // Make sure the target is a Fiducial target.
        if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
          lowestAmbiguityScore = targetPoseAmbiguity;
          lowestAmbiguityTarget = target;
        }
      }

      if (lowestAmbiguityTarget == null) {
        return Optional.empty();
      }

      Optional<Pose3d> targetPosition =
          m_fieldLayout.getTagPose(lowestAmbiguityTarget.getFiducialId());

      if (targetPosition.isEmpty()) {
        DriverStation.reportWarning(
            "[VisionPoseEstimator] The found tag was not within the FieldLayout.", false);
        return Optional.empty();
      }

      return Optional.of(
          new EstimatedRobotPose(
              targetPosition
                  .get()
                  .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                  .transformBy(m_robotToCamera.inverse()),
              pipelineTimestamp));
    } else {
      Optional<Matrix<N3, N3>> cameraMatrixOpt = m_camera.getCameraMatrix();
      Optional<Matrix<N5, N1>> distCoeffsOpt = m_camera.getDistCoeffs();

      boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();

      if (!hasCalibData) {
        return Optional.empty();
      }

      ArrayList<TargetCorner> visCorners = new ArrayList<>();
      ArrayList<AprilTag> knownVisTags = new ArrayList<>();

      for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
        visCorners.addAll(target.getDetectedCorners());

        Optional<Pose3d> tagPoseOpt = m_fieldLayout.getTagPose(target.getFiducialId());

        if (tagPoseOpt.isEmpty()) {
          DriverStation.reportWarning(
              "[VisionPoseEstimator] The found tag was not within the FieldLayout.", false);
          continue;
        }

        Pose3d tagPose = tagPoseOpt.get();

        // actual layout poses of visible tags -- not exposed, so have to recreate
        knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));
      }

      // multi-target solvePNP
      Matrix<N3, N3> cameraMatrix = cameraMatrixOpt.get();
      Matrix<N5, N1> distCoeffs = distCoeffsOpt.get();

      PNPResults pnpResults =
          VisionEstimation.estimateCamPosePNP(cameraMatrix, distCoeffs, visCorners, knownVisTags);
      Pose3d bestRobotPose =
          new Pose3d()
              .plus(pnpResults.best) // field-to-camera
              .plus(m_robotToCamera.inverse()); // field-to-robot

      return Optional.of(new EstimatedRobotPose(bestRobotPose, pipelineTimestamp));
    }
  }
}
