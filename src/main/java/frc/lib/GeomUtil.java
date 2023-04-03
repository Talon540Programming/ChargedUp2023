package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;

public class GeomUtil {

  public static Pose2d invertAngle(Pose2d pose) {
    return pose.transformBy(new Transform2d(new Translation2d(), Rotation2d.fromDegrees(180)));
  }
}
