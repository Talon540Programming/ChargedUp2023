package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;

public class GeomUtil {
  private GeomUtil() {}

  public static Pose2d toPose2d(Pose3d pose) {
    return new Pose2d(pose.getX(), pose.getY(), Rotation2d.fromRadians(pose.getRotation().getZ()));
  }

  public static Pose3d toPose3d(Pose2d pose) {
    return new Pose3d(pose);
  }
}
