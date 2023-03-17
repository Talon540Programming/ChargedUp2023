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
   * Calculate the angle of the arm from the position of a point on the arm in the Robot Coordinate
   * System.
   *
   * @param pointOnArm the point's position as a {@link Pose3d} object in the RCS.
   * @return angle of the arm in radians.
   */
  public double calculateArmAngleFromPointOnArm(Pose3d pointOnArm) {
    double dist = fulcrumPosition.getDistance(pointOnArm.getTranslation());
    return Math.asin((pointOnArm.getZ() - fulcrumPosition.getZ()) / dist);
  }

  /**
   * Calculate the Moment of Inertia of the arm based on its length, mass of the arm, and mass of
   * the effector.
   *
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   * @return estimated MoI of the Arm and Effector
   */
  public double calculateMoI(double length, double mass) {
    // double armMass = RobotDimensions.Arm.kArmMassKg; // Mass of just the arm.
    // double armDistance =
    //     calculateArmCenterOfMassDistance(
    //         pivotToEffectorMeters); // Distance from the pivot point to the center of mass of the
    // // arm.
    // double counterweightMass =
    //     RobotDimensions.Arm.kCounterweightMassKg; // Mass of the counterweight
    // double counterweightDistance =
    //     RobotDimensions.Arm
    //         .kCounterweightCenterOfMassOffsetMeters; // Distance from the pivot point to the center
    // // of mass of the counterweight.
    // double effectorMass = RobotDimensions.Effector.kEffectorMassKg; // Mass of the effector
    // double effectorDistance =
    //     calculateThirdExtrusionDistance(pivotToEffectorMeters)
    //         + RobotDimensions.Effector
    //             .kEffectorCenterOfMassOffsetMeters; // Distance from the pivot point to the center
    // // of mass of the effector.

    // return (1.0 / 3.0) * armMass * Math.pow(armDistance, 2)
    //     + counterweightMass * Math.pow(counterweightDistance, 2)
    //     + effectorMass * Math.pow(effectorDistance, 2);

    return (1.0 / 3.0) * mass * Math.pow(length, 2);
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

    return effectorPose.getZ() <= RobotDimensions.Effector.kWidthMeters / 2
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

    return effectorPose.getZ() <= RobotDimensions.Effector.kWidthMeters / 2
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
        (fulcrumPosition.getZ() - (RobotDimensions.Effector.kWidthMeters / 2))
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
            (fulcrumPosition.getZ() - (RobotDimensions.Effector.kWidthMeters / 2))
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
