package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.FieldConstants;
import frc.robot.constants.RobotDimensions;
import frc.robot.constants.RobotLimits;

public class ArmState {
  // Set of Preset ArmStates
  public static final ArmState IDLE =
      new ArmState(Rotation2d.fromDegrees(90), RobotLimits.kMinArmLengthMeters);

  public static final ArmState FLOOR = new ArmState(Rotation2d.fromRadians(-0.357393), 0.52705);
  public static final ArmState SINGLE_SUBSTATION =
      new ArmState(Rotation2d.fromRadians(0.42339), 0.52705);
  public static final ArmState DOUBLE_SUBSTATION =
      new ArmState(Rotation2d.fromRadians(0.48851), 1.19380);

  public static final ArmState SCORE_HIGH_CUBE;
  public static final ArmState SCORE_MID_CUBE;
  public static final ArmState SCORE_MID_CONE;
  public static final ArmState SCORE_HYBRID;

  static {
    // Resolve the ideal ArmStates in case they were changed and because I am lazy to redo them by
    // hand
    Pose2d idealRobotPose =
        new Pose2d(
            FieldConstants.kGridWallXCoordinateMeters
                + RobotDimensions.Drivetrain.kDrivetrainLengthBumpersMeters / 2.0
                + FieldConstants.kGridScoreDistanceAwayMeters,
            0,
            Rotation2d.fromDegrees(180));

    SCORE_HIGH_CUBE =
        Constants.Arm.kArmKinematics
            .calculateArmState(
                idealRobotPose,
                FieldConstants.kGrid[0][1].getIdealEffectorPose().getTranslation(),
                RobotDimensions.Effector.kEffectorCubeOffsetMeters)
            .invert();
    SCORE_MID_CONE =
        Constants.Arm.kArmKinematics
            .calculateArmState(
                idealRobotPose,
                FieldConstants.kGrid[1][0].getIdealEffectorPose().getTranslation(),
                RobotDimensions.Effector.kEffectorCubeOffsetMeters)
            .invert();
    SCORE_MID_CUBE =
        Constants.Arm.kArmKinematics
            .calculateArmState(
                idealRobotPose,
                FieldConstants.kGrid[1][1].getIdealEffectorPose().getTranslation(),
                RobotDimensions.Effector.kEffectorCubeOffsetMeters)
            .invert();
    SCORE_HYBRID =
        Constants.Arm.kArmKinematics
            .calculateArmState(
                idealRobotPose,
                FieldConstants.kGrid[2][0].getIdealEffectorPose().getTranslation(),
                RobotDimensions.Effector.kEffectorCubeOffsetMeters)
            .invert();
  }

  public final Rotation2d Angle;
  public final double VelocityRadiansPerSecond;
  public final double PivotToEffectorDistanceMeters;

  /**
   * Create an ArmState object.
   *
   * @param angleRad angle of the arm.
   * @param armVelocityRadPerSecond velocity of the arm in radians per second.
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   */
  public ArmState(
      Rotation2d angleRad, double armVelocityRadPerSecond, double pivotToEffectorMeters) {
    this.Angle = Rotation2d.fromRadians(angleRad.getRadians() % (2 * Math.PI));
    this.VelocityRadiansPerSecond = armVelocityRadPerSecond;
    this.PivotToEffectorDistanceMeters =
        MathUtil.clamp(
            pivotToEffectorMeters,
            RobotLimits.kMinArmLengthMeters,
            RobotLimits.kMaxArmLengthMeters);
  }

  /**
   * Create an ArmState object. Assumed the arm shouldn't be moving (velocity of 0).
   *
   * @param angleRad angle of the arm in radians.
   * @param pivotToEffectorMeters distance from the pivot to the origin (beginning point) of the
   *     effector.
   */
  public ArmState(Rotation2d angleRad, double pivotToEffectorMeters) {
    this(angleRad, 0, pivotToEffectorMeters);
  }

  /**
   * Return a copy of the ArmState with the angle reflected across the y-axis.
   *
   * @return reflected angle.
   */
  public ArmState invert() {
    return new ArmState(
        new Rotation2d(-Angle.getCos(), Angle.getSin()),
        VelocityRadiansPerSecond,
        PivotToEffectorDistanceMeters);
  }

  @Override
  public boolean equals(Object obj) {
    if (obj instanceof ArmState other) {
      return Math.abs(Angle.getRadians() - other.Angle.getRadians()) < 0.05
          && Math.abs(PivotToEffectorDistanceMeters - other.PivotToEffectorDistanceMeters) < 0.01;
    }
    return false;
  }

  @Override
  public String toString() {
    return String.format(
        "ArmState(Position Rad: %.2f, Velocity Rad/s: %.2f, Length Meters: %.2f",
        Angle.getRadians(), VelocityRadiansPerSecond, PivotToEffectorDistanceMeters);
  }
}
