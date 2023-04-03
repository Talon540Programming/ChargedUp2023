package frc.lib;

import edu.wpi.first.math.geometry.Pose2d;

public class GeomUtil {

  public static Pose2d invertAngle(Pose2d pose) {
    return new Pose2d(pose.getTranslation(), pose.getRotation().unaryMinus());
  }
}
