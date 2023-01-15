package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.estimator.DifferentialDrivePoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.constants.HardwareDevices;
import org.talon540.sensors.TalonFXMechanism;

public class DrivetrainBase extends SubsystemBase {
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

  private final MotorControllerGroup m_leftGroup;
  private final MotorControllerGroup m_rightGroup;

  private final TalonFXMechanism kLeftSensor =
      new TalonFXMechanism(
          m_frontLeft.getSensorCollection(),
          Drivetrain.kWheelRadiusMeters,
          Drivetrain.kDrivetrainGearRatio);
  private final TalonFXMechanism kRightSensors =
      new TalonFXMechanism(
          m_frontRight.getSensorCollection(),
          Drivetrain.kWheelRadiusMeters,
          Drivetrain.kDrivetrainGearRatio);

  private final PIDController m_leftPIDController =
      new PIDController(Drivetrain.PIDValues.kP, Drivetrain.PIDValues.kI, Drivetrain.PIDValues.kD);
  private final PIDController m_rightPIDController =
      new PIDController(Drivetrain.PIDValues.kP, Drivetrain.PIDValues.kI, Drivetrain.PIDValues.kD);

  private final SimpleMotorFeedforward m_driveFeedForward = new SimpleMotorFeedforward(0, 0);

  private final DifferentialDrivePoseEstimator m_driveOdometry;

  public DrivetrainBase() {
    this.m_frontLeft.setNeutralMode(Drivetrain.kDrivetrainNeutralMode);
    this.m_frontRight.setNeutralMode(Drivetrain.kDrivetrainNeutralMode);
    this.m_backLeft.setNeutralMode(Drivetrain.kDrivetrainNeutralMode);
    this.m_backRight.setNeutralMode(Drivetrain.kDrivetrainNeutralMode);

    this.m_leftGroup = new MotorControllerGroup(m_frontLeft, m_backLeft);
    this.m_rightGroup = new MotorControllerGroup(m_frontRight, m_backRight);

    this.m_leftGroup.setInverted(true);

    this.m_gyro.reset();
    this.kLeftSensor.resetEncoder();
    this.kRightSensors.resetEncoder();

    this.m_driveOdometry =
        new DifferentialDrivePoseEstimator(
            Drivetrain.kDrivetrainKinematics,
            m_gyro.getRotation2d(),
            kLeftSensor.getPosition(),
            kRightSensors.getPosition(),
            new Pose2d());
  }

  @Override
  public void periodic() {
    updateOdometry();
  }

  public void setPercent(double leftPercent, double rightPercent) {
    m_leftGroup.set(leftPercent);
    m_rightGroup.set(rightPercent);
  }

  public void setWheelSpeeds(DifferentialDriveWheelSpeeds speeds) {
    double leftFeedForward = m_driveFeedForward.calculate(speeds.leftMetersPerSecond);
    double rightFeedForward = m_driveFeedForward.calculate(speeds.rightMetersPerSecond);

    double leftOutput =
        m_leftPIDController.calculate(kLeftSensor.getLinearVelocity(), speeds.leftMetersPerSecond);
    double rightOutput =
        m_rightPIDController.calculate(
            kRightSensors.getLinearVelocity(), speeds.rightMetersPerSecond);

    m_leftGroup.setVoltage(leftOutput + leftFeedForward);
    m_rightGroup.setVoltage(rightOutput + rightFeedForward);
  }

  public void setChassisSpeed(ChassisSpeeds chassisSpeed) {
    setWheelSpeeds(Drivetrain.kDrivetrainKinematics.toWheelSpeeds(chassisSpeed));
  }

  public void setFromForces(double linearSpeed, double angularSpeed) {
    setChassisSpeed(new ChassisSpeeds(linearSpeed, 0, angularSpeed));
  }

  public void addEstimatedPose(Pose2d estimatedPose, double timestampSeconds) {
    m_driveOdometry.addVisionMeasurement(estimatedPose, timestampSeconds);
  }

  public void addEstimatedPose(Pose3d estimatedPose, double timestampSeconds) {
    addEstimatedPose(estimatedPose.toPose2d(), timestampSeconds);
  }

  private void updateOdometry() {
    m_driveOdometry.update(
        m_gyro.getRotation2d(), kLeftSensor.getPosition(), kRightSensors.getPosition());
  }
}
