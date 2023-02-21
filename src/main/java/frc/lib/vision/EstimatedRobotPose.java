package frc.lib.vision;

import edu.wpi.first.math.geometry.Pose3d;

public record EstimatedRobotPose(Pose3d robotPose, double timestampSeconds) {}
