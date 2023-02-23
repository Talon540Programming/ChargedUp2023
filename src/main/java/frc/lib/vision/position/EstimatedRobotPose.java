package frc.lib.vision.position;

import edu.wpi.first.math.geometry.Pose3d;

public class EstimatedRobotPose {
    public Pose3d robotPose;
    public double timestampSeconds;

    public EstimatedRobotPose(Pose3d robotPose, double timestampSeconds) {
        this.robotPose = robotPose;
        this.timestampSeconds = timestampSeconds;
    }
}
