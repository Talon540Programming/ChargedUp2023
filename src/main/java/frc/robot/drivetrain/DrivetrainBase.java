package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.drive.RobotDriveBase;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.constants.HardwareDevices;
import org.talon540.sensors.TalonFXMechanism;

public class DrivetrainBase extends SubsystemBase {
  // region: config
  // region: hardware
  public final WPI_Pigeon2 m_gyro =
      new WPI_Pigeon2(
          HardwareDevices.kRobotGyroConfig.id, HardwareDevices.kRobotGyroConfig.controller);

  private final WPI_TalonFX m_leftLeader =
      new WPI_TalonFX(
          HardwareDevices.Drivetrain.kLeftLeader.id,
          HardwareDevices.Drivetrain.kLeftLeader.controller);
  private final WPI_TalonFX m_rightLeader =
      new WPI_TalonFX(
          HardwareDevices.Drivetrain.kRightLeader.id,
          HardwareDevices.Drivetrain.kRightLeader.controller);
  private final WPI_TalonFX m_leftFollower =
      new WPI_TalonFX(
          HardwareDevices.Drivetrain.kLeftFollower.id,
          HardwareDevices.Drivetrain.kLeftFollower.controller);
  private final WPI_TalonFX m_rightFollower =
      new WPI_TalonFX(
          HardwareDevices.Drivetrain.kRightFollower.id,
          HardwareDevices.Drivetrain.kRightFollower.controller);

  // endregion

  private final MotorControllerGroup m_leftGroup =
      new MotorControllerGroup(m_leftLeader, m_leftFollower);
  private final MotorControllerGroup m_rightGroup =
      new MotorControllerGroup(m_rightLeader, m_rightFollower);

  private final TalonFXMechanism kLeftSensor =
      new TalonFXMechanism(
          m_leftLeader.getSensorCollection(),
          Drivetrain.kWheelRadiusMeters,
          Drivetrain.kDrivetrainGearRatio);
  private final TalonFXMechanism kRightSensor =
      new TalonFXMechanism(
          m_rightLeader.getSensorCollection(),
          Drivetrain.kWheelRadiusMeters,
          Drivetrain.kDrivetrainGearRatio);

  private final PIDController m_leftPIDController =
      new PIDController(
          Drivetrain.ControlValues.WheelSpeed.kP,
          Drivetrain.ControlValues.WheelSpeed.kI,
          Drivetrain.ControlValues.WheelSpeed.kD);
  private final PIDController m_rightPIDController =
      new PIDController(
          Drivetrain.ControlValues.WheelSpeed.kP,
          Drivetrain.ControlValues.WheelSpeed.kI,
          Drivetrain.ControlValues.WheelSpeed.kD);

  private final SimpleMotorFeedforward m_driveFeedForward =
      new SimpleMotorFeedforward(
          Drivetrain.ControlValues.WheelSpeed.kS,
          Drivetrain.ControlValues.WheelSpeed.kV,
          Drivetrain.ControlValues.WheelSpeed.kA);

  private final DifferentialDrivePoseEstimator m_poseEstimator;
  // endregion

  /** Create the drivetrain subsystem. Configures and resets encoder. */
  public DrivetrainBase() {
    this.m_leftGroup.setInverted(true);

    zeroHeading();

    setNeutralMode(Drivetrain.kDrivetrainDefaultNeutralMode);

    resetEncoders();

    this.m_poseEstimator =
        new DifferentialDrivePoseEstimator(
            Drivetrain.kDrivetrainKinematics,
            m_gyro.getRotation2d(),
            kLeftSensor.getPosition(),
            kRightSensor.getPosition(),
            new Pose2d());
  }

  @Override
  public void periodic() {
    updateOdometry();

    SmartDashboard.putNumberArray(
        "gyro_data", new double[] {m_gyro.getRoll(), m_gyro.getPitch(), m_gyro.getYaw()});
  }

  /**
   * Get the current wheel speeds of the drivetrain.
   *
   * @return current wheel speeds.
   */
  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        kLeftSensor.getLinearVelocity(), kRightSensor.getLinearVelocity());
  }

  /**
   * Set the wheel speeds of the drivetrain using a PID loop + Feed Forward control (closed loop).
   * The controllers must be reset prior to using this method via {@link
   * DrivetrainBase#resetControllers()}.
   *
   * @param speeds speeds to set.
   * @param leftAcceleration acceleration of the left side for the Feed Forward loop.
   * @param rightAcceleration acceleration of the right side for the Feed Forward loop.
   */
  public void setFromWheelSpeeds(
      DifferentialDriveWheelSpeeds speeds, double leftAcceleration, double rightAcceleration) {
    double leftFeedForward =
        m_driveFeedForward.calculate(speeds.leftMetersPerSecond, leftAcceleration);
    double rightFeedForward =
        m_driveFeedForward.calculate(speeds.rightMetersPerSecond, rightAcceleration);

    double leftOutput =
        m_leftPIDController.calculate(kLeftSensor.getLinearVelocity(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(
            kRightSensor.getLinearVelocity(), speeds.rightMetersPerSecond);

    tankDriveVoltage(leftOutput + leftFeedForward, rightOutput + rightFeedForward);
  }

  /**
   * Set the wheel speeds of the drivetrain using a PID loop + Feed Forward control (closed loop).
   * The controllers must be reset prior to using this method via {@link
   * DrivetrainBase#resetControllers()}. Assumes an acceleration of 0.
   *
   * @param speeds speeds to set.
   */
  public void setFromWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    setFromWheelSpeeds(speeds, 0, 0);
  }

  /**
   * Get the current Chassis Speed of the drivetrain.
   *
   * @return current Chassis Speed of the drivetrain.
   */
  public ChassisSpeeds getChassisSpeed() {
    return Drivetrain.kDrivetrainKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  /**
   * Set the speed of the drivetrain from a {@link ChassisSpeeds} object using a PID loop + Feed
   * Forward control (closed loop). The controllers must be reset prior to using this method via
   * {@link DrivetrainBase#resetControllers()}. Uses an acceleration of 0.
   *
   * @param chassisSpeed chassis speed to set.
   */
  public void setFromChassisSpeed(ChassisSpeeds chassisSpeed) {
    setFromWheelSpeeds(Drivetrain.kDrivetrainKinematics.toWheelSpeeds(chassisSpeed));
  }

  /**
   * Set the speed of the drivetrain from linear and rotational forces.
   *
   * @param linearSpeed linear velocity to apply in meters per second.
   * @param angularSpeed rotational velocity to apply in radians per second.
   */
  public void setFromForces(double linearSpeed, double angularSpeed) {
    setFromChassisSpeed(new ChassisSpeeds(linearSpeed, 0, angularSpeed));
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

    leftPercent = MathUtil.clamp(leftPercent, -1.0, 1.0);
    rightPercent = MathUtil.clamp(rightPercent, -1.0, 1.0);

    m_leftGroup.set(leftPercent);
    m_rightGroup.set(rightPercent);
  }

  /**
   * Drive the robot based on output voltage to set to each side of the drivetrain. This is useful
   * when controlling the drivetrain using closed loop control via Feed Forward control. <b>This
   * already checks the battery voltage</b>.
   *
   * @param leftVolts output voltage for the left side of the drivetrain.
   * @param rightVolts output voltage for the right side of the drivetrain.
   */
  public void tankDriveVoltage(double leftVolts, double rightVolts) {
    m_leftGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(rightVolts);
  }

  /** Stop the drivetrain from moving (sets speed to 0). */
  public void stop() {
    m_leftGroup.stopMotor();
    m_rightGroup.stopMotor();
  }

  /**
   * Get the estimated current position of the robot as a {@link Pose2d} object. Uses odometry
   * combined with a fused Kalman filter with Vision measurements via the {@link
   * DifferentialDrivePoseEstimator} class.
   *
   * @return estimated position of the robot.
   */
  public Pose2d getRobotPosition() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Reset the estimated position of the robot to a position.
   *
   * @param position position to reset the robot to.
   */
  public void resetOdometry(Pose2d position) {
    resetEncoders();
    m_poseEstimator.resetPosition(
        m_gyro.getRotation2d(), kLeftSensor.getPosition(), kRightSensor.getPosition(), position);
  }

  /**
   * Add a Vision measurement to the Pose Estimator that can be used to account for system noise.
   *
   * @param estimatedPose the estimated position of the robot.
   * @param timestampSeconds the robot start time timestamp of the estimated position.
   */
  public void addEstimatedPose(Pose2d estimatedPose, double timestampSeconds) {
    m_poseEstimator.addVisionMeasurement(estimatedPose, timestampSeconds);
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

  /** Update the odometry with the current encoder and gyro data. */
  private void updateOdometry() {
    m_poseEstimator.update(
        m_gyro.getRotation2d(), kLeftSensor.getPosition(), kRightSensor.getPosition());
  }

  /** Zero the heading of the gyro. */
  public void zeroHeading() {
    m_gyro.reset();
  }

  /** Reset the drivetrain encoders to 0. */
  public void resetEncoders() {
    this.kLeftSensor.resetEncoder();
    this.kRightSensor.resetEncoder();
  }

  /** Reset the PID controllers used for closed loop control. */
  public void resetControllers() {
    m_leftPIDController.reset();
    m_rightPIDController.reset();
  }

  /**
   * Set the neutral mode of the drivetrain motors.
   *
   * @param mode Neutral mode to set the drivetrain motors to.
   */
  public void setNeutralMode(NeutralMode mode) {
    m_leftLeader.setNeutralMode(mode);
    m_rightLeader.setNeutralMode(mode);
    m_leftFollower.setNeutralMode(mode);
    m_rightFollower.setNeutralMode(mode);
  }

  /** Reset the neutral mode of the drivetrain to the default mode. */
  public void resetNeutralMode() {
    setNeutralMode(Drivetrain.kDrivetrainDefaultNeutralMode);
  }
}
