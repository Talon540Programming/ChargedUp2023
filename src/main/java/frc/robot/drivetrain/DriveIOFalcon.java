package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_Pigeon2;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.constants.Constants;
import frc.robot.constants.Flags.NeutralMode;
import frc.robot.constants.HardwareDevices;
import org.talon540.sensors.TalonFXMechanism;

public class DriveIOFalcon implements DriveIO {
  private final WPI_Pigeon2 m_gyro;

  private final WPI_TalonFX m_leftLeader;
  private final WPI_TalonFX m_rightLeader;
  private final WPI_TalonFX m_leftFollower;
  private final WPI_TalonFX m_rightFollower;

  private final TalonFXMechanism m_leftSensors;
  private final TalonFXMechanism m_rightSensors;

  public DriveIOFalcon() {
    switch (Constants.kCurrentMode) {
      case PROTO -> {
        m_gyro =
            new WPI_Pigeon2(
                HardwareDevices.PROTO.kRobotGyroConfig.id,
                HardwareDevices.PROTO.kRobotGyroConfig.controller);
        m_leftLeader =
            new WPI_TalonFX(
                HardwareDevices.PROTO.Drivetrain.kLeftLeader.id,
                HardwareDevices.PROTO.Drivetrain.kLeftLeader.controller);
        m_leftFollower =
            new WPI_TalonFX(
                HardwareDevices.PROTO.Drivetrain.kLeftFollower.id,
                HardwareDevices.PROTO.Drivetrain.kLeftFollower.controller);
        m_rightLeader =
            new WPI_TalonFX(
                HardwareDevices.PROTO.Drivetrain.kRightLeader.id,
                HardwareDevices.PROTO.Drivetrain.kRightLeader.controller);
        m_rightFollower =
            new WPI_TalonFX(
                HardwareDevices.PROTO.Drivetrain.kRightFollower.id,
                HardwareDevices.PROTO.Drivetrain.kRightFollower.controller);
      }
      case COMP -> {
        m_gyro =
            new WPI_Pigeon2(
                HardwareDevices.COMP.kRobotGyroConfig.id,
                HardwareDevices.COMP.kRobotGyroConfig.controller);
        m_leftLeader =
            new WPI_TalonFX(
                HardwareDevices.COMP.Drivetrain.kLeftLeader.id,
                HardwareDevices.COMP.Drivetrain.kLeftLeader.controller);
        m_leftFollower =
            new WPI_TalonFX(
                HardwareDevices.COMP.Drivetrain.kLeftFollower.id,
                HardwareDevices.COMP.Drivetrain.kLeftFollower.controller);
        m_rightLeader =
            new WPI_TalonFX(
                HardwareDevices.COMP.Drivetrain.kRightLeader.id,
                HardwareDevices.COMP.Drivetrain.kRightLeader.controller);
        m_rightFollower =
            new WPI_TalonFX(
                HardwareDevices.COMP.Drivetrain.kRightFollower.id,
                HardwareDevices.COMP.Drivetrain.kRightFollower.controller);
      }
      default -> throw new RuntimeException(
          "Shouldn't be using this IO system if running on a SIM robot");
    }

    m_leftSensors =
        new TalonFXMechanism(
            m_leftLeader.getSensorCollection(),
            Constants.Drivetrain.kWheelRadiusMeters,
            Constants.Drivetrain.kDrivetrainGearRatio);

    m_rightSensors =
        new TalonFXMechanism(
            m_rightLeader.getSensorCollection(),
            Constants.Drivetrain.kWheelRadiusMeters,
            Constants.Drivetrain.kDrivetrainGearRatio);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 0, 0);

    m_leftLeader.configAllSettings(config);
    m_rightLeader.configAllSettings(config);

    setNeutralMode(Constants.Drivetrain.kDrivetrainDefaultNeutralMode);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_leftLeader.setInverted(true);
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightLeader.setInverted(false);
    m_rightFollower.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    inputs.LeftPosition = getLeftPositionMeters();
    inputs.LeftVelocity = getLeftVelocityMetersPerSecond();
    inputs.RightPosition = getRightPositionMeters();
    inputs.RightVelocity = getRightVelocityMetersPerSecond();
    inputs.GyroYaw = m_gyro.getYaw();
  }

  @Override
  public double getLeftPositionMeters() {
    return m_leftSensors.getPosition();
  }

  @Override
  public double getLeftVelocityMetersPerSecond() {
    return m_leftSensors.getLinearVelocity();
  }

  @Override
  public double getRightPositionMeters() {
    return m_rightSensors.getPosition();
  }

  @Override
  public double getRightVelocityMetersPerSecond() {
    return m_rightSensors.getLinearVelocity();
  }

  @Override
  public Rotation2d getHeading() {
    return m_gyro.getRotation2d();
  }

  @Override
  public void resetEncoders() {
    m_leftSensors.resetEncoder();
    m_rightSensors.resetEncoder();
  }

  @Override
  public void zeroHeading() {
    m_gyro.reset();
  }

  @Override
  public double getRate() {
    return m_gyro.getRate();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void setPercent(double leftPercent, double rightPercent) {
    leftPercent = MathUtil.clamp(leftPercent, -1, 1);
    rightPercent = MathUtil.clamp(rightPercent, -1, 1);

    m_leftLeader.set(ControlMode.PercentOutput, leftPercent);
    m_rightLeader.set(TalonFXControlMode.PercentOutput, rightPercent);
  }

  @Override
  public void setNeutralMode(NeutralMode mode) {
    switch (mode) {
      case COAST -> {
        m_leftLeader.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        m_rightLeader.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        m_leftFollower.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
        m_rightFollower.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Coast);
      }
      case BRAKE -> {
        m_leftLeader.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        m_rightLeader.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        m_leftFollower.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
        m_rightFollower.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
      }
    }
  }
}
