package frc.robot.drivetrain;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.constants.Flags;
import org.littletonrobotics.junction.Logger;


public class DriveBase extends SubsystemBase {
  public final DriveIO m_io;
  private final DriveIOInputsAutoLogged m_inputs = new DriveIOInputsAutoLogged();
  private final DifferentialDrivePoseEstimator m_odometry;

  public DriveBase(DriveIO driveIO) {
    this.m_io = driveIO;

    m_io.resetEncoders();
    m_io.zeroHeading();

    m_io.setNeutralMode(Drivetrain.kDrivetrainDefaultNeutralMode);

    this.m_odometry =
        new DifferentialDrivePoseEstimator(
            Drivetrain.kDrivetrainKinematics,
            new Rotation2d(),
            m_io.getLeftPositionMeters(),
            m_io.getRightPositionMeters(),
            new Pose2d());
  }

  @Override
  public void periodic() {
    m_io.updateInputs(m_inputs);
    Logger.getInstance().processInputs("Drive", m_inputs);

    // Data in DriveIO is automatically logged using AutoLog

    m_odometry.update(m_io.getHeading(), getLeftPositionMeters(), getRightPositionMeters());
    Logger.getInstance().recordOutput("Odometry", getPose());
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

    m_io.setPercent(leftPercent, rightPercent);
  }

  /**
   * Drive the robot based on output voltage to set to each side of the drivetrain. This is
   useful
   * when controlling the drivetrain using closed loop control via Feed Forward control. <b>This
   * already checks the battery voltage</b>.
   *
   * @param leftVoltage output voltage for the left side of the drivetrain.
   * @param rightVoltage output voltage for the right side of the drivetrain.
   */
  public void tankDriveVoltage(double leftVoltage, double rightVoltage) {
    m_io.setVoltage(leftVoltage, rightVoltage);
  }

  /**
   * Get the position of the left side of the drivetrain. This already accounts for gear ratio and wheel radius.
   *
   * @return distance in meters.
   */
  public double getLeftPositionMeters() {
    return m_io.getLeftPositionMeters();
  }

  /**
   * Get the velocity of the left side of the drivetrain.
   *
   * @return velocity in meters per second.
   */
  public double getLeftVelocityMetersPerSecond() {
    return m_io.getLeftVelocityMetersPerSecond();
  }

  /**
   * Get the position of the right side of the drivetrain. This already accounts for gear ratio and wheel radius.
   *
   * @return distance in meters.
   */
  public double getRightPositionMeters() {
    return m_io.getRightPositionMeters();
  }

  /**
   * Get the velocity of the right side of the drivetrain.
   *
   * @return velocity in meters per second.
   */
  public double getRightVelocityMetersPerSecond() {
    return m_io.getRightVelocityMetersPerSecond();
  }

  /**
   * Get the current speed of the drivetrain wheels.
   *
   * @return current drivetrain wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(getLeftVelocityMetersPerSecond(), getRightVelocityMetersPerSecond());
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
    m_io.setVoltage(0, 0);
  }

  /**
   * Get the estimated current position of the robot as a {@link Pose2d} object. Uses odometry
   * combined with a fused Kalman filter with Vision measurements via the {@link
   * DifferentialDrivePoseEstimator} class.
   *
   * @return estimated position of the robot.
   */
  public Pose2d getPose() {
    return m_odometry.getEstimatedPosition();
  }

  /**
   * Reset the estimated position of the robot to a position.
   *
   * @param position position to reset the robot to.
   */
  public void resetPosition(Pose2d position) {
    m_io.resetEncoders();
    m_odometry.resetPosition(
            m_io.getHeading(), getLeftPositionMeters(), getRightPositionMeters(), position);
  }

  /**
   * Add a Vision measurement to the Pose Estimator that can be used to account for system noise.
   *
   * @param estimatedPose the estimated position of the robot.
   * @param timestampSeconds the robot start time timestamp of the estimated position.
   */
  public void addEstimatedPose(Pose2d estimatedPose, double timestampSeconds) {
    m_odometry.addVisionMeasurement(estimatedPose, timestampSeconds);
  }

  /**
   * Add a Vision measurement to the Pose Estimator that can be used to account for system noise.
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
  public void setNeutralMode(Flags.NeutralMode mode) {
    m_io.setNeutralMode(mode);
  }

  /** Reset the neutral mode of the drivetrain to the default mode. */
  public void resetNeutralMode() {
    setNeutralMode(Drivetrain.kDrivetrainDefaultNeutralMode);
  }
}
