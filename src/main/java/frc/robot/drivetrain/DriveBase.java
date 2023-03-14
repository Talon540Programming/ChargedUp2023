package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.ArmStateManager;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Drivetrain;
import org.littletonrobotics.junction.Logger;

public class DriveBase extends SubsystemBase {
  private final DriveIO m_driveIO;
  public final DriveIOInputsAutoLogged m_driveInputs = new DriveIOInputsAutoLogged();

  private final DifferentialDrivePoseEstimator m_odometry;

  /**
   * Create the drivetrain subsystem.
   *
   * @param driveIO Drive interface to use.
   */
  public DriveBase(DriveIO driveIO) {
    this.m_driveIO = driveIO;

    m_driveIO.resetEncoders();
    m_driveIO.resetHeading();

    resetNeutralMode();

    m_driveIO.updateInputs(m_driveInputs);

    this.m_odometry =
        new DifferentialDrivePoseEstimator(
            Drivetrain.kDrivetrainKinematics,
            m_driveIO.getHeading(),
            m_driveInputs.LeftPositionMeters,
            m_driveInputs.RightPositionMeters,
            new Pose2d());
  }

  @Override
  public void periodic() {
    m_driveIO.updateInputs(m_driveInputs);
    Logger.getInstance().processInputs("Drive", m_driveInputs);

    Logger.getInstance().recordOutput("heading", m_driveIO.getHeading().toString());

    // Data in DriveIO is automatically logged using AutoLog. Odometry is handled in subsystem.
    m_odometry.update(
        Rotation2d.fromRadians(m_driveInputs.GyroPitchRad),
        m_driveInputs.LeftPositionMeters,
        m_driveInputs.RightPositionMeters);
    Logger.getInstance().recordOutput("Drive/Odometry", getPosition());

    ArmStateManager.getInstance().updateRobotPitch(m_driveInputs.GyroPitchRad);
  }

  /**
   * Get the average distance traveled by the left and right sides of the drivetrain.
   *
   * @return distance traveled.
   */
  public double getLinearDistanceTraveled() {
    return (m_driveInputs.LeftPositionMeters + m_driveInputs.RightPositionMeters) / 2.0;
  }

  /**
   * Drive the robot using standard tank drive from left and right percent. The {@link
   * RobotDriveBase#kDefaultDeadband} deadband value is used. <b>Input values are not squared</b>
   *
   * @param leftPercent percent to apply to the left side of the drivetrain [-1, 1].
   * @param rightPercent percent to apply to the right side of the drivetrain [-1, 1].
   */
  public void tankDrivePercent(double leftPercent, double rightPercent) {
    leftPercent = MathUtil.applyDeadband(leftPercent, RobotDriveBase.kDefaultDeadband);
    rightPercent = MathUtil.applyDeadband(rightPercent, RobotDriveBase.kDefaultDeadband);

    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1, 1);

    m_driveIO.setVoltage(leftPercent * 12.0, rightPercent * 12.0);
  }

  /**
   * Drive the robot using an Arcade style fashion. The forward percent refers to speed forward and
   * backwards while the rotation percent causes the drivetrain to spin in a counterclockwise
   * positive fashion.
   *
   * @param forwardPercent The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rotationPercent The robot's rotation rate around the Z axis [-1.0..1.0].
   *     Counterclockwise is positive.
   */
  public void arcadeDrivePercent(double forwardPercent, double rotationPercent) {
    forwardPercent = MathUtil.applyDeadband(forwardPercent, RobotDriveBase.kDefaultDeadband);
    rotationPercent = MathUtil.applyDeadband(rotationPercent, RobotDriveBase.kDefaultDeadband);

    forwardPercent = MathUtil.clamp(forwardPercent, -1, 1);
    rotationPercent = MathUtil.clamp(rotationPercent, -1, 1);

    double leftSpeed = forwardPercent - rotationPercent;
    double rightSpeed = forwardPercent + rotationPercent;

    double greaterInput = Math.max(Math.abs(forwardPercent), Math.abs(rotationPercent));
    double lesserInput = Math.min(Math.abs(forwardPercent), Math.abs(rotationPercent));

    if (greaterInput == 0) m_driveIO.setVoltage(0, 0);

    double saturatedInput = (greaterInput + lesserInput) / greaterInput;
    leftSpeed /= saturatedInput;
    rightSpeed /= saturatedInput;

    m_driveIO.setVoltage(leftSpeed * 12.0, rightSpeed * 12.0);
  }

  /**
   * Drive the robot based on output voltage to set to each side of the drivetrain. This is useful
   * when controlling the drivetrain using closed loop control via Feed Forward control. <b>This
   * already checks the battery voltage</b>.
   *
   * @param leftVoltage output voltage for the left side of the drivetrain.
   * @param rightVoltage output voltage for the right side of the drivetrain.
   */
  public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
    leftVoltage = MathUtil.clamp(leftVoltage, -12.0, 12.0);
    rightVoltage = MathUtil.clamp(rightVoltage, -12.0, 12.0);

    m_driveIO.setVoltage(leftVoltage, rightVoltage);
  }

  /**
   * Get the current speed of the drivetrain wheels.
   *
   * @return current drivetrain wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        m_driveInputs.LeftVelocityMetersPerSecond, m_driveInputs.RightVelocityMetersPerSecond);
  }

  /**
   * Get the current speed of the robot as a {@link ChassisSpeeds} object.
   *
   * @return current robot speed.
   */
  public ChassisSpeeds getChassisSpeed() {
    return Drivetrain.kDrivetrainKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /** Stop the drivetrain from moving (sets speed to 0). */
  public void stop() {
    m_driveIO.setVoltage(0, 0);
  }

  /**
   * Get the estimated current position of the robot as a {@link Pose2d} object. Uses odometry
   * combined with a fused Kalman filter with Vision measurements via the {@link
   * DifferentialDrivePoseEstimator} class.
   *
   * @return estimated position of the robot.
   */
  public Pose2d getPosition() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Reset the estimated position of the robot to a position.
   *
   * @param position position to reset the robot to.
   */
  public void resetPosition(Pose2d position) {
    m_odometry.resetPosition(
        m_driveIO.getHeading(),
        m_driveInputs.LeftPositionMeters,
        m_driveInputs.RightPositionMeters,
        position);
  }

  /**
   * Add a Vision measurement to the Pose Estimator that can be used to account for system noise.
   * Make sure this isn't a duplicate timestamp position or else the values of the filter will be
   * clogged.
   *
   * @param estimatedPose the estimated position of the robot.
   * @param timestampSeconds the robot start time timestamp of the estimated position.
   */
  public void addEstimatedPose(Pose2d estimatedPose, double timestampSeconds) {
    m_odometry.addVisionMeasurement(estimatedPose, timestampSeconds);
  }

  /**
   * Add a Vision measurement to the Pose Estimator that can be used to account for system noise.
   * Make sure this isn't a duplicate timestamp position or else the values of the filter will be
   * clogged.
   *
   * @param estimatedPose the estimated position of the robot.
   * @param timestampSeconds the robot start time timestamp of the estimated position.
   */
  public void addEstimatedPose(Pose3d estimatedPose, double timestampSeconds) {
    addEstimatedPose(estimatedPose.toPose2d(), timestampSeconds);
  }

  /**
   * Set the neutral mode of the drivetrain motors.
   *
   * @param mode Neutral mode to set the drivetrain motors to.
   */
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_driveIO.setNeutralMode(mode);
  }

  /** Reset the neutral mode of the drivetrain to the default mode. */
  public void resetNeutralMode() {
    setNeutralMode(Constants.Drivetrain.kDrivetrainDefaultNeutralMode);
  }

  /** Brake the drivetrain motors then stop them, Useful to avoid a collision. */
  public void emergencyBrake() {
    setNeutralMode(Constants.NeutralMode.BRAKE);
    stop();
  }

  /**
   * Check if the Gyroscope is at a level plane.
   *
   * @return whether the gyroscope is level.
   */
  public boolean isLevel() {
    return Math.abs(m_driveInputs.GyroPitchRad)
        < Math.toRadians(Drivetrain.kRobotStabilizationToleranceDegrees);
  }
}
