package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc.robot.constants.RobotDimensions;

/**
 * Kinematics helper class that can help calculate the state of the Arm for either its current or
 * desired state. All length measurements should be from the pivot point (center of the rotation
 * shaft) to the origin of the effector (beginning of the effector).
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
   * Get the position of the fulcrum as a {@link Translation3d} object.
   *
   * @return position of the fulcrum.
   */
  public Translation3d getFulcrumPose() {
    return fulcrumPosition;
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
   * Calculate the distance from the end of the second extrusion to the pivot point.
   *
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @return estimated distance from the end of the second extrusion to the pivot point.
   */
  public double calculateSecondExtrusionDistance(double pivotToEffectorMeters) {
    double minLength = RobotDimensions.Arm.kFirstExtrusionLengthMeters;
    double maxLength =
        RobotDimensions.Arm.kFirstExtrusionLengthMeters
            + RobotDimensions.Arm.kSecondExtrusionLengthMeters;

    return MathUtil.clamp(pivotToEffectorMeters, minLength, maxLength) - Units.inchesToMeters(1);
  }

  /**
   * Calculate the position of the end of the second extrusion / stage of the telescoping arm.
   *
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @param armAngleRad the angle made between the arm and the plane bisecting the fulcrum in
   *     radians.
   * @return estimated position of the end of the second extrusion in the Robot Coordinate System.
   */
  public Pose3d calculateSecondExtrusionPose(double pivotToEffectorMeters, double armAngleRad) {
    return calculatePose(calculateSecondExtrusionDistance(pivotToEffectorMeters), armAngleRad);
  }

  /**
   * Calculate the distance from the end of the third extrusion to the pivot point.
   *
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @return estimated distance from the end of the third extrusion to the pivot point.
   */
  public double calculateThirdExtrusionDistance(double pivotToEffectorMeters) {
    double minLength = RobotDimensions.Arm.kFirstExtrusionLengthMeters;
    double maxLength =
        RobotDimensions.Arm.kFirstExtrusionLengthMeters
            + RobotDimensions.Arm.kSecondExtrusionLengthMeters
            + RobotDimensions.Arm.kThirdExtrusionLengthMeters;

    return MathUtil.clamp(pivotToEffectorMeters, minLength, maxLength);
  }

  /**
   * Calculate the position of the end of the third extrusion / stage of the telescoping arm.
   *
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @param armAngleRad the angle made between the arm and the plane bisecting the fulcrum in
   *     radians.
   * @return estimated position of the end of the third extrusion in the Robot Coordinate System.
   */
  public Pose3d calculateThirdExtrusionPose(double pivotToEffectorMeters, double armAngleRad) {
    return calculatePose(calculateThirdExtrusionDistance(pivotToEffectorMeters), armAngleRad);
  }

  /**
   * Calculate the Moment of Inertia of the arm based on its length, mass of the arm, and mass of
   * the effector. The estimation of the MoI is found <a
   * href="https://www.desmos.com/calculator/a6wx6jikow">here</a>.
   *
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @return estimated MoI of the Arm and Effector
   */
  public static double calculateMoI(double pivotToEffectorMeters) {
    return 3.08887 * Math.pow(pivotToEffectorMeters, 2)
        + 2.10357 * pivotToEffectorMeters
        - 0.0945233;
  }

  /**
   * Estimates the distance from the pivot point to the center of mass of just the arm (without the
   * counterweight and the effector). <a href="https://www.desmos.com/calculator/y82f7lckoo">...</a>
   *
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @return estimated distance in meters.
   */
  public static double calculateArmCenterOfMassDistance(double pivotToEffectorMeters) {
    double pivotToEffectorInches = Units.metersToInches(pivotToEffectorMeters);

    if (pivotToEffectorInches < 40) {
      return Units.inchesToMeters(0.465 * pivotToEffectorInches - 0.72744048);
    } else {
      return Units.inchesToMeters(0.209 * pivotToEffectorInches + 9.5161);
    }
  }

  /**
   * Check if an arm's dimensions would breach the extension limits.
   *
   * @param totalLengthMeters total length of the arm from the pivot point to the end of the
   *     effector.
   * @param armAngleRad angle of the arm.
   * @return whether the extension limit would be breached.
   */
  public boolean wouldBreakExtensionLimit(double totalLengthMeters, double armAngleRad) {
    Pose3d endPose = calculatePose(totalLengthMeters, armAngleRad);
    double horizontalOffset =
        RobotDimensions.Drivetrain.kDrivetrainLengthMeters / 2.0 + Units.inchesToMeters(48.0);

    return Math.abs(endPose.getX()) > horizontalOffset || endPose.getZ() > Units.inchesToMeters(78);
  }

  /**
   * Calculate the max arm length that would still be within the extension limit given the angle of
   * the arm.
   *
   * @param armAngleRadians angle of the arm in radians.
   * @return max length of the arm.
   */
  public double maxArmAndEffectorLength(double armAngleRadians) {
    Rotation2d armAngle = Rotation2d.fromRadians(armAngleRadians);
    double horizontalOffset =
        RobotDimensions.Drivetrain.kDrivetrainLengthMeters / 2.0 + Units.inchesToMeters(48.0);
    double verticalOffset = Units.inchesToMeters(78.0) - fulcrumPosition.getZ();

    return Math.min(
        Math.abs(horizontalOffset / armAngle.getCos()),
        Math.abs(verticalOffset / armAngle.getSin()));
  }

  /**
   * Check if a given length and angle would cause the arm to get close to, or hit the floor in the
   * front of the robot.
   *
   * @param totalLengthMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @param armAngleRadians angle of the arm in radians.
   * @return whether the arm would intersect in front of the robot.
   */
  public boolean wouldIntersectForward(double totalLengthMeters, double armAngleRadians) {
    Pose3d effectorPose = calculatePose((totalLengthMeters), armAngleRadians);

    return effectorPose.getZ() <= RobotDimensions.Effector.kWidthMeters / 2 + 0.05
        && effectorPose.getX() > 0;
  }

  /**
   * Check if a given length and angle would cause the arm to get close to, or hit the floor in the
   * back of the robot.
   *
   * @param totalLengthMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @param armAngleRadians angle of the arm in radians.
   * @return whether the arm would intersect in the back of the robot.
   */
  public boolean wouldIntersectRear(double totalLengthMeters, double armAngleRadians) {
    Pose3d effectorPose = calculatePose((totalLengthMeters), armAngleRadians);

    return effectorPose.getZ() <= RobotDimensions.Effector.kWidthMeters / 2 + 0.05
        && effectorPose.getX() < 0;
  }

  /**
   * Get the angle of the arm of the lowest non-intersecting point at the front of the robot. This
   * is usually the floor.
   *
   * @param totalLengthMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @return estimated angle of the arm.
   */
  public double lowestForwardAngle(double totalLengthMeters) {
    return -Math.asin(
        (fulcrumPosition.getZ() - (RobotDimensions.Effector.kWidthMeters / 2  + 0.05))
            / (totalLengthMeters));
  }

  /**
   * Get the angle of the arm of the lowest non-intersecting point at the back of the robot. This is
   * usually the floor.
   *
   * @param totalLengthMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @return estimated angle of the arm.
   */
  public double lowestRearAngle(double totalLengthMeters) {
    return Math.PI
        + Math.asin(
            (fulcrumPosition.getZ() - (RobotDimensions.Effector.kWidthMeters / 2 + 0.05))
                / (totalLengthMeters));
  }

  /**
   * Calculate the pose of a point on the arm from its distance from the fulcrum and the angle
   * formed.
   *
   * @param lengthMeters magnitude in meters.
   * @param angleRad angle formed in radians.
   * @return estimated position of the point on the arm.
   */
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
