package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotDimensions;
import frc.robot.constants.RobotLimits;

/**
 * Kinematics helper class that can help calculate the state of the Arm for either its current or
 * desired state.
 */
public class ArmKinematics {
  private final Translation3d fulcrumPosition;

  /**
   * Create an ArmKinematics object that can be used to predict the position and state of the arm
   * from measured data.
   *
   * @param fulcrumPosition position of the arm's fulcrum as a {@link Translation3d} object.
   */
  public ArmKinematics(Translation3d fulcrumPosition) {
    this.fulcrumPosition = fulcrumPosition;
  }

  /**
   * Calculate the position of the end of the first extrusion / stage of the telescoping arm.
   *
   * @param armAngleRad the angle made between the arm and the plane bisecting the fulcrum in
   *     radians.
   * @return estimated position of the end of the first extrusion in the Robot Coordinate System.
   */
  public Pose3d calculateFirstExtrusionPose(double armAngleRad) {
    return calculatePose(RobotDimensions.Arm.kFirstExtrusionLengthMeters, armAngleRad);
  }

  /**
   * Calculate the position of the end of the second extrusion / stage of the telescoping arm.
   *
   * @param totalLengthMeters distance from the fulcrum to the end of the effector (including the
   *     effector itself).
   * @param armAngleRad the angle made between the arm and the plane bisecting the fulcrum in
   *     radians.
   * @return estimated position of the end of the second extrusion in the Robot Coordinate System.
   */
  public Pose3d calculateSecondExtrusionPose(double totalLengthMeters, double armAngleRad) {
    // Distance from the fulcrum to the end of the third extrusion.
    totalLengthMeters -= RobotDimensions.Effector.kLengthMeters;

    double minLength = RobotDimensions.Arm.kFirstExtrusionLengthMeters + Units.inchesToMeters(2.5);
    double maxLength =
        RobotDimensions.Arm.kFirstExtrusionLengthMeters
            + RobotDimensions.Arm.kSecondExtrusionLengthMeters;

    totalLengthMeters = MathUtil.clamp(totalLengthMeters, minLength, maxLength);

    return calculatePose(totalLengthMeters, armAngleRad);
  }

  /**
   * Calculate the position of the end of the third extrusion / stage of the telescoping arm.
   *
   * @param totalLengthMeters distance from the fulcrum to the end of the effector (including the
   *     effector itself).
   * @param armAngleRad the angle made between the arm and the plane bisecting the fulcrum in
   *     radians.
   * @return estimated position of the end of the third extrusion in the Robot Coordinate System.
   */
  public Pose3d calculateThirdExtrusionPose(double totalLengthMeters, double armAngleRad) {
    totalLengthMeters -= RobotDimensions.Effector.kLengthMeters;

    double minLength = RobotDimensions.Arm.kFirstExtrusionLengthMeters + Units.inchesToMeters(2.5);
    double maxLength =
        RobotDimensions.Arm.kFirstExtrusionLengthMeters
            + RobotDimensions.Arm.kSecondExtrusionLengthMeters
            + RobotDimensions.Arm.kThirdExtrusionLengthMeters;

    totalLengthMeters =
        MathUtil.clamp(totalLengthMeters, minLength, maxLength) + Units.inchesToMeters(0.5);

    return calculatePose(totalLengthMeters, armAngleRad);
  }

  /**
   * Calculate the position of the effector (object at the end of the arm);
   *
   * @param distanceMeters distance from the fulcrum to the end of the effector (including the
   *     effector itself).
   * @param armAngleRad the angle made between the arm and the plane bisecting the fulcrum in
   *     radians.
   * @return estimated position of the effector in the Robot Coordinate System.
   */
  public Pose3d calculateEffectorPose(double totalLengthMeters, double armAngleRad) {
    totalLengthMeters =
        MathUtil.clamp(
            totalLengthMeters, RobotLimits.kMinArmLengthMeters, RobotLimits.kMaxArmLengthMeters);

    return calculatePose(totalLengthMeters, armAngleRad);
  }

  /**
   * Calculate the angle of the arm from the position of a point on the arm in the Robot Coordinate
   * System.
   *
   * @param effectorPosition the point's position as a {@link Pose3d} object in the RCS.
   * @return angle of the arm in radians.
   */
  public double calculateArmAngleFromPointOnArm(Pose3d pointOnArm) {
    double dist = fulcrumPosition.getDistance(pointOnArm.getTranslation());
    return Math.asin((pointOnArm.getZ() - fulcrumPosition.getZ()) / dist);
  }

  /**
   * Get the position of the fulcrum as a {@link Translation3d} object.
   *
   * @return position of the fulcrum.
   */
  public Translation3d getFulcrumPose() {
    return fulcrumPosition;
  }

  private Pose3d calculatePose(double lengthMeters, double angleRad) {
    Rotation2d armAngleRotation2d = Rotation2d.fromRadians(angleRad);

    return new Pose3d(
        fulcrumPosition.plus(
            new Translation3d(
                lengthMeters * armAngleRotation2d.getCos(),
                0,
                lengthMeters * armAngleRotation2d.getSin())),
        new Rotation3d(0, -angleRad, 0));
  }
}
