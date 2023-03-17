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

    totalLengthMeters = MathUtil.clamp(totalLengthMeters, minLength, maxLength) - Units.inchesToMeters(1);

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

    double minLength = RobotDimensions.Arm.kFirstExtrusionLengthMeters + Units.inchesToMeters(3.5);
    double maxLength =
        RobotDimensions.Arm.kFirstExtrusionLengthMeters
            + RobotDimensions.Arm.kSecondExtrusionLengthMeters
            + RobotDimensions.Arm.kThirdExtrusionLengthMeters;
    
    totalLengthMeters =
        MathUtil.clamp(totalLengthMeters, minLength, maxLength);

    return calculatePose(totalLengthMeters, armAngleRad);
  }

  /**
   * Calculate the position of the effector (object at the end of the arm);
   *
   * @param totalLengthMeters distance from the fulcrum to the end of the effector (including the
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
   * @param totalLengthMeters length of the arm and effector in meters.
   * @param massKg mass items of the arm. Include the effector in this.
   * @return estimated MoI of the Arm and Effector
   */
  public double calculateMoI(double totalLengthMeters, double... massKg) {
    // TODO, make this more accurate to the actual arm
    double totalMassKg = 0;

    for (double mass : massKg) totalMassKg += mass;

    return (1.0 / 3.0) * totalMassKg * Math.pow(totalLengthMeters, 2);
  }

  /**
   * Check if a given length and angle would cause the arm to get close to, or hit the floor in the
   * front of the robot.
   *
   * @param totalLengthMeters length of the arm and effector in meters.
   * @param armAngleRadians angle of the arm in radians.
   * @return whether the arm would intersect in front of the robot.
   */
  public boolean wouldIntersectForward(double totalLengthMeters, double armAngleRadians) {
    Pose3d effectorPose = calculatePose(totalLengthMeters, armAngleRadians);

    return effectorPose.getZ() <= RobotDimensions.Effector.kWidthMeters / 2 && effectorPose.getX() > 0;
  }

  /**
   * Check if a given length and angle would cause the arm to get close to, or hit the floor in the
   * back of the robot.
   *
   * @param totalLengthMeters length of the arm and effector in meters.
   * @param armAngleRadians angle of the arm in radians.
   * @return  whether the arm would intersect in the back of the robot.
   */
  public boolean wouldIntersectRear(double totalLengthMeters, double armAngleRadians) {
    Pose3d effectorPose = calculatePose(totalLengthMeters, armAngleRadians);

    return effectorPose.getZ() <= RobotDimensions.Effector.kWidthMeters / 2 && effectorPose.getX() < 0;
  }

  /**
   * Get the angle of the arm of the lowest non-intersecting point at the front of the robot. This
   * is usually the floor.
   *
   * @param totalLengthMeters length of the arm in meters.
   * @return estimated angle of the arm.
   */
  public double lowestForwardAngle(double totalLengthMeters) {
    return -Math.asin((fulcrumPosition.getZ() - (RobotDimensions.Effector.kWidthMeters / 2)) / totalLengthMeters);
  }

  /**
   * Get the angle of the arm of the lowest non-intersecting point at the back of the robot. This is
   * usually the floor.
   *
   * @param totalLengthMeters length of the arm in meters.
   * @return estimated angle of the arm.
   */
  public double lowestRearAngle(double totalLengthMeters) {
    return Math.PI + Math.asin((fulcrumPosition.getZ() - (RobotDimensions.Effector.kWidthMeters / 2)) / totalLengthMeters);
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
