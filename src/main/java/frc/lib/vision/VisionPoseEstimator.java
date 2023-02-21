package frc.lib.vision;

import edu.wpi.first.apriltag.AprilTag;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.numbers.N5;
import edu.wpi.first.wpilibj.DriverStation;
import java.util.*;
import org.photonvision.estimation.PNPResults;
import org.photonvision.estimation.VisionEstimation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;
import org.photonvision.targeting.TargetCorner;

/**
 * Use MultiTag SolvePNP to calculate the composite position of the robot from multiple AprilTag
 * targets. If MultiTag is unable to find any targets, or can't be used, targets with the lowest
 * calculated <a href="https://arxiv.org/pdf/1812.00287.pdf">ambiguity</a> are used.
 */
public class VisionPoseEstimator {
  private final AprilTagFieldLayout fieldLayout;
  private List<PhotonCamera> m_cameras = new ArrayList<>();
  // Create a cache of timestamps from previous data. It is important not to clog the Kalman Filter
  // with duplicate data as it will screw up the values.
  private List<Double> m_timestamps = new ArrayList<>();

  /**
   * Create the Pose Estimator from the field layout to use and a list of PhotonCameras
   *
   * @param fieldLayout FieldLayout of AprilTags to use.
   * @param cameras cameras to use.
   */
  public VisionPoseEstimator(AprilTagFieldLayout fieldLayout, PhotonCamera... cameras) {
    this.fieldLayout = fieldLayout;
    m_cameras.addAll(List.of(cameras));
  }

  /**
   * Add a camera that the PoseEstimator can use.
   *
   * @param camera camera to add.
   */
  public void addCamera(PhotonCamera camera) {
    m_cameras.add(camera);
    m_timestamps.add(-1.0);
  }

  /**
   * Get the estimated position of the robot from each camera.
   *
   * @return HashMap of all targets where the key is the camera name and the value is an Optional of
   *     a {@link EstimatedRobotPose}
   */
  public HashMap<String, Optional<EstimatedRobotPose>> getRobotPose() {
    if (m_cameras.isEmpty()) {
      DriverStation.reportWarning("[VisionPoseEstimator] No cameras were set", false);
      // No targets, return empty map
      return new HashMap<>();
    }

    if (m_cameras.size() != m_timestamps.size()) {
      DriverStation.reportWarning(
          "[VisionPoseEstimator] Camera and Timestamp Cache indexes dont match", false);
      return new HashMap<>();
    }

    HashMap<String, Optional<EstimatedRobotPose>> results = new HashMap<>();

    for (int i = 0; i < m_cameras.size(); i++) {
      PhotonCamera camera = m_cameras.get(i);
      PhotonPipelineResult pipelineResult = camera.getLatestResult();
      double pipelineTimestamp = pipelineResult.getTimestampSeconds();

      // The pipeline is invalid or there are no targets
      if (pipelineTimestamp < 0 || !pipelineResult.hasTargets()) {
        results.put(camera.getName(), Optional.empty());
        continue;
      }

      double timestampCache = m_timestamps.get(i);

      // If there is a timestamp, make sure that the time delta between the two states is
      // significant
      if (timestampCache > 0 && Math.abs(timestampCache - pipelineTimestamp) < 1e-6) {
        results.put(camera.getName(), Optional.empty());
        continue;
      }

      // Set the timestamp cache to the current timestamp
      m_timestamps.set(i, pipelineTimestamp);

      /* ======================== */
      if (pipelineResult.getTargets().size() < 2) {
        // There are only 2 targets, use the target with the lowest ambiguity
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

        // Although there are confirmed to be targets, none of them may be fiducial
        // targets.
        if (lowestAmbiguityTarget == null) {
          results.put(camera.getName(), Optional.empty());
          continue;
        }

        int targetFiducialId = lowestAmbiguityTarget.getFiducialId();
        Optional<Pose3d> targetPosition = fieldLayout.getTagPose(targetFiducialId);

        if (targetPosition.isEmpty()) {
          DriverStation.reportWarning(
              "[VisionPoseEstimator] The found tag was not within the FieldLayout.", false);
          results.put(camera.getName(), Optional.empty());
          continue;
        }

        results.put(
            camera.getName(),
            Optional.of(
                new EstimatedRobotPose(
                    targetPosition
                        .get()
                        .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                        .transformBy(camera.getRobotToCamera().inverse()),
                    pipelineTimestamp)));
      } else {
        ArrayList<TargetCorner> visCorners = new ArrayList<>();
        ArrayList<AprilTag> knownVisTags = new ArrayList<>();

        for (PhotonTrackedTarget target : pipelineResult.getTargets()) {
          visCorners.addAll(target.getDetectedCorners());

          Optional<Pose3d> tagPoseOpt = fieldLayout.getTagPose(target.getFiducialId());
          if (tagPoseOpt.isEmpty()) {
            DriverStation.reportWarning(
                "[VisionPoseEstimator] The found tag was not within the FieldLayout.", false);
            continue;
          }

          Pose3d tagPose = tagPoseOpt.get();

          // actual layout poses of visible tags -- not exposed, so have to recreate
          knownVisTags.add(new AprilTag(target.getFiducialId(), tagPose));
        }

        Optional<Matrix<N3, N3>> cameraMatrixOpt = camera.getCameraMatrix();
        Optional<Matrix<N5, N1>> distCoeffsOpt = camera.getDistCoeffs();
        boolean hasCalibData = cameraMatrixOpt.isPresent() && distCoeffsOpt.isPresent();

        // multi-target solvePNP
        if (hasCalibData) {
          Matrix<N3, N3> cameraMatrix = cameraMatrixOpt.get();
          Matrix<N5, N1> distCoeffs = distCoeffsOpt.get();
          PNPResults pnpResults =
              VisionEstimation.estimateCamPosePNP(
                  cameraMatrix, distCoeffs, visCorners, knownVisTags);
          Pose3d best =
              new Pose3d()
                  .plus(pnpResults.best) // field-to-camera
                  .plus(camera.getRobotToCamera().inverse()); // field-to-robot

          results.put(
              camera.getName(), Optional.of(new EstimatedRobotPose(best, pipelineTimestamp)));
        } else {
          results.put(camera.getName(), Optional.empty());
        }
      }
    }

    return results;
  }
}
