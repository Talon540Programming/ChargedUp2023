package frc.lib.vision;

import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.wpilibj.DriverStation;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import java.util.*;

/**
 * Use MultiTag SolvePNP to calculate the composite position of the robot from multiple AprilTag targets.
 * If MultiTag is unable to find any targets, or can't be used, targets with the lowest calculated <a href="https://arxiv.org/pdf/1812.00287.pdf">ambiguity</a> are used.
 */
public class VisionPoseEstimator {
    private final AprilTagFieldLayout fieldLayout;
    private List<PhotonCamera> m_cameras = new ArrayList<>();
    // Create a cache of timestamps from previous data. It is important not to clog the Kalman Filter with duplicate data as it will screw up the values.
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
     * @return HashMap of all targets where the key is the camera name and the value is an Optional of a {@link EstimatedRobotPose}
     */
    public HashMap<String, Optional<EstimatedRobotPose>> getRobotPose() {
        if(m_cameras.isEmpty()) {
            DriverStation.reportWarning("[VisionPoseEstimator] No cameras were set", false);
            // No targets, return empty map
            return new HashMap<>();
        }

        if(m_cameras.size() != m_timestamps.size()) {
            DriverStation.reportWarning("[VisionPoseEstimator] Camera and Timestamp Cache indexes dont match", false);
            return new HashMap<>();
        }

        HashMap<String, Optional<EstimatedRobotPose>> results = new HashMap<>();

        for(int i = 0; i < m_cameras.size(); i++) { // TODO, is this n - 1 or just n, I don't remember
            PhotonCamera camera = m_cameras.get(i);
            PhotonPipelineResult pipelineResult = camera.getLatestResult();
            double pipelineTimestamp = pipelineResult.getTimestampSeconds();

            // The pipeline is invalid or there are no targets
            if(pipelineTimestamp < 0 || !pipelineResult.hasTargets()) {
                results.put(camera.getName(), Optional.empty());
                continue;
            }

            double timestampCache = m_timestamps.get(i);

            // If there is a timestamp, make sure that the time delta between the two states is significant
            if(timestampCache > 0 && Math.abs(timestampCache - pipelineTimestamp) < 1e-6) {
                results.put(camera.getName(), Optional.empty());
                continue;
            }

            // Set the timestamp cache to the current timestamp
            m_timestamps.set(i, pipelineTimestamp);

            /* ======================== */
            if(pipelineResult.targets.size() < 2) {
                // There are only 2 targets, use the target with the lowest ambiguity


                PhotonTrackedTarget lowestAmbiguityTarget = null;
                double lowestAmbiguityScore = 10;

                for (PhotonTrackedTarget target : pipelineResult.targets) {
                    double targetPoseAmbiguity = target.getPoseAmbiguity();
                    // Make sure the target is a Fiducial target.
                    if (targetPoseAmbiguity != -1 && targetPoseAmbiguity < lowestAmbiguityScore) {
                        lowestAmbiguityScore = targetPoseAmbiguity;
                        lowestAmbiguityTarget = target;
                    }
                }

                // Although there are confirmed to be targets, none of them may be fiducial
                // targets.
                if(lowestAmbiguityTarget == null) {
                    results.put(camera.getName(), Optional.empty());
                    continue;
                }

                int targetFiducialId = lowestAmbiguityTarget.getFiducialId();
                Optional<Pose3d> targetPosition = fieldLayout.getTagPose(targetFiducialId);

                if(targetPosition.isEmpty()) {
                    DriverStation.reportWarning("[VisionPoseEstimator] The found tag was not within the FieldLayout.", false);
                    results.put(camera.getName(), Optional.empty());
                    continue;
                }

                results.put(camera.getName(), Optional.of(
                        new EstimatedRobotPose(targetPosition
                                .get()
                                .transformBy(lowestAmbiguityTarget.getBestCameraToTarget().inverse())
                                .transformBy(camera.getRobotToCamera().inverse()),
                                pipelineTimestamp
                )));
            } else {

            }
        }

        return results;
    }
}
