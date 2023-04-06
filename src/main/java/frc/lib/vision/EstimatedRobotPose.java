package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;

/** Represents the estimated position of the robot at a given time. */
public class EstimatedRobotPose {
  public Pose3d robotPose;
  public double timestampSeconds;

  /**
   * Create an EstimatedRobotPose.
   *
   * @param robotPose position of the robot.
   * @param timestampSeconds timestamp of the estimated position.
   */
  public EstimatedRobotPose(Pose3d robotPose, double timestampSeconds) {
    this.robotPose = robotPose;
    this.timestampSeconds = timestampSeconds;
  }
}
