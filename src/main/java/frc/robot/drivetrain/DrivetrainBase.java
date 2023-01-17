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
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.constants.HardwareDevices;
import org.talon540.sensors.TalonFXMechanism;

public class DrivetrainBase extends SubsystemBase {
  // region: config
  // region: hardware
  private final WPI_Pigeon2 m_gyro =
      new WPI_Pigeon2(
          HardwareDevices.kRobotGyroConfig.id, HardwareDevices.kRobotGyroConfig.controller);

  private final WPI_TalonFX m_frontLeft =
      new WPI_TalonFX(
          HardwareDevices.Drivetrain.kFrontLeftConfig.id,
          HardwareDevices.Drivetrain.kFrontLeftConfig.controller);
  private final WPI_TalonFX m_frontRight =
      new WPI_TalonFX(
          HardwareDevices.Drivetrain.kFrontRightConfig.id,
          HardwareDevices.Drivetrain.kFrontRightConfig.controller);
  private final WPI_TalonFX m_backLeft =
      new WPI_TalonFX(
          HardwareDevices.Drivetrain.kBackLeftConfig.id,
          HardwareDevices.Drivetrain.kBackLeftConfig.controller);
  private final WPI_TalonFX m_backRight =
      new WPI_TalonFX(
          HardwareDevices.Drivetrain.kBackRightConfig.id,
          HardwareDevices.Drivetrain.kBackRightConfig.controller);

  // endregion

  private final MotorControllerGroup m_leftGroup = new MotorControllerGroup(m_frontLeft, m_backLeft);
  private final MotorControllerGroup m_rightGroup = new MotorControllerGroup(m_frontRight, m_backRight);

  private final TalonFXMechanism kLeftSensor =
      new TalonFXMechanism(
          m_frontLeft.getSensorCollection(),
          Drivetrain.kWheelRadiusMeters,
          Drivetrain.kDrivetrainGearRatio);
  private final TalonFXMechanism kRightSensor =
      new TalonFXMechanism(
          m_frontRight.getSensorCollection(),
          Drivetrain.kWheelRadiusMeters,
          Drivetrain.kDrivetrainGearRatio);

  private final PIDController m_leftPIDController =
      new PIDController(
          Drivetrain.ControlValues.kP, Drivetrain.ControlValues.kI, Drivetrain.ControlValues.kD);
  private final PIDController m_rightPIDController =
      new PIDController(
          Drivetrain.ControlValues.kP, Drivetrain.ControlValues.kI, Drivetrain.ControlValues.kD);

  private final SimpleMotorFeedforward m_driveFeedForward =
      new SimpleMotorFeedforward(Drivetrain.ControlValues.kS, Drivetrain.ControlValues.kV);

  private final DifferentialDrivePoseEstimator m_driveOdometry;
  // endregion

  public DrivetrainBase() {
    this.m_leftGroup.setInverted(true);

    setNeutralMode(Drivetrain.kDrivetrainNeutralMode);

    zeroHeading();
    resetEncoders();

    this.m_driveOdometry =
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
  }

  public DifferentialDriveWheelSpeeds getWheelSpeeds() {
    return new DifferentialDriveWheelSpeeds(kLeftSensor.getLinearVelocity(), kRightSensor.getLinearVelocity());
  }

  public void setFromWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedForward = m_driveFeedForward.calculate(speeds.leftMetersPerSecond);
    double rightFeedForward = m_driveFeedForward.calculate(speeds.rightMetersPerSecond);

    double leftOutput =
        m_leftPIDController.calculate(kLeftSensor.getLinearVelocity(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(
                kRightSensor.getLinearVelocity(), speeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(leftOutput + leftFeedForward);
    m_rightGroup.setVoltage(rightOutput + rightFeedForward);
  }

  public ChassisSpeeds getChassisSpeed() {
    return Drivetrain.kDrivetrainKinematics.toChassisSpeeds(getWheelSpeeds());
  }

  public void setFromChassisSpeed(ChassisSpeeds chassisSpeed) {
    setFromWheelSpeeds(Drivetrain.kDrivetrainKinematics.toWheelSpeeds(chassisSpeed));
  }

  public void setFromForces(double linearSpeed, double angularSpeed) {
    setFromChassisSpeed(new ChassisSpeeds(linearSpeed, 0, angularSpeed));
  }

  public void tankDrivePercent(double leftPercent, double rightPercent) {
    leftPercent = MathUtil.applyDeadband(leftPercent,
            RobotDriveBase.kDefaultDeadband
    );
    rightPercent = MathUtil.applyDeadband(rightPercent,
            RobotDriveBase.kDefaultDeadband
    );

    leftPercent = MathUtil.clamp(leftPercent, -1.0, 1.0);
    leftPercent = MathUtil.clamp(leftPercent, -1.0, 1.0);

    m_leftGroup.set(leftPercent);
    m_rightGroup.set(rightPercent);
  }

  public void tankDriveVoltage(double leftVolts, double rightVolts) {
    m_rightGroup.setVoltage(leftVolts);
    m_rightGroup.setVoltage(rightVolts);
  }

  public Pose2d getRobotPosition() {
    return m_driveOdometry.getEstimatedPosition();
  }

  public void addEstimatedPose(Pose2d estimatedPose, double timestampSeconds) {
    m_driveOdometry.addVisionMeasurement(estimatedPose, timestampSeconds);
  }

  public void addEstimatedPose(Pose3d estimatedPose, double timestampSeconds) {
    addEstimatedPose(estimatedPose.toPose2d(), timestampSeconds);
  }

  private void updateOdometry() {
    m_driveOdometry.update(m_gyro.getRotation2d(), kLeftSensor.getPosition(), kRightSensor.getPosition());
  }

  public void zeroHeading() {
    m_gyro.reset();
  }

  public void resetEncoders() {
    this.kLeftSensor.resetEncoder();
    this.kRightSensor.resetEncoder();
  }

  public void setNeutralMode(NeutralMode mode) {
    m_frontLeft.setNeutralMode(mode);
    m_frontRight.setNeutralMode(mode);
    m_backLeft.setNeutralMode(mode);
    m_backRight.setNeutralMode(mode);
  }
}
