package frc.robot.arm;

import edu.wpi.first.math.geometry.*;

public class ArmKinematics {
  private final Pose3d fulcrumPosition;

  public ArmKinematics(Pose3d fulcrumPosition) {
    this.fulcrumPosition = fulcrumPosition;
  }

  /**
   * Calculate the position of the effector (object at the end of the arm);
   *
   * @param effectorDistanceFromFulcrum distance from the fulcrum to the end of the effector
   *     (including the effector itself).
   * @param armAngle the angle made between the arm and the plane bisecting the fulcrum.
   * @return effective position of the effector in the Robot Coordinate System.
   */
  public Pose3d calculateEffectorPose(double effectorDistanceFromFulcrum, double armAngle) {
    Rotation2d armAngleRotation2d = Rotation2d.fromRadians(armAngle);

    return fulcrumPosition.transformBy(
        new Transform3d(
            new Translation3d(
                effectorDistanceFromFulcrum * armAngleRotation2d.getCos(),
                0,
                effectorDistanceFromFulcrum * armAngleRotation2d.getSin()),
            new Rotation3d(0, 0, 0)));
  }

  /**
   * Calculate the angle of the arm from the position of the effector in the Robot Coordinate
   * System.
   *
   * @param effectorPosition the effector's position as a {@link Pose3d} object in the RCS.
   * @return angle of the arm in radians.
   */
  public double calculateArmAngleFromEffectorPose(Pose3d effectorPosition) {
    double dist = fulcrumPosition.getTranslation().getDistance(effectorPosition.getTranslation());
    return Math.asin((effectorPosition.getZ() - fulcrumPosition.getZ()) / dist);
  }
}
