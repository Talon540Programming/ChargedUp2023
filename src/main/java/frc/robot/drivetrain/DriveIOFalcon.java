// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;
import org.talon540.hardware.CANDeviceConfig;
import org.talon540.sensors.TalonFXMechanism;

/**
 * {@link DriveIO} used for interfacing with a series of {@link WPI_TalonFX} devices for a
 * Differential Drivetrain.
 */
public class DriveIOFalcon implements DriveIO {
  private final WPI_TalonFX m_leftLeader;
  private final WPI_TalonFX m_rightLeader;
  private final WPI_TalonFX m_leftFollower;
  private final WPI_TalonFX m_rightFollower;

  private final TalonFXMechanism m_leftSensors;
  private final TalonFXMechanism m_rightSensors;

  private final boolean leftSideInverted;
  private final boolean leftSensorInverted;
  private final boolean rightSideInverted;
  private final boolean rightSensorInverted;

  /** Create the DriveIO. */
  public DriveIOFalcon(DriveIOFalconConfig driveConfig) {
    this.leftSideInverted = driveConfig.leftSideInverted;
    this.leftSensorInverted = driveConfig.leftSensorInverted;
    this.rightSideInverted = driveConfig.rightSideInverted;
    this.rightSensorInverted = driveConfig.rightSensorInverted;

    m_leftLeader = new WPI_TalonFX(driveConfig.leftLeader.id, driveConfig.leftFollower.controller);
    m_leftFollower =
        new WPI_TalonFX(driveConfig.leftFollower.id, driveConfig.leftFollower.controller);
    m_rightLeader = new WPI_TalonFX(driveConfig.rightLeader.id, driveConfig.rightLeader.controller);
    m_rightFollower =
        new WPI_TalonFX(driveConfig.rightFollower.id, driveConfig.rightFollower.controller);

    m_leftSensors =
        new TalonFXMechanism(
            m_leftLeader.getSensorCollection(),
            driveConfig.wheelRadiusMeters,
            driveConfig.gearRatio);

    m_rightSensors =
        new TalonFXMechanism(
            m_rightLeader.getSensorCollection(),
            driveConfig.wheelRadiusMeters,
            driveConfig.gearRatio);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 0, 0);

    m_leftLeader.configAllSettings(config);
    m_rightLeader.configAllSettings(config);

    setNeutralMode(Constants.Drivetrain.kDrivetrainDefaultNeutralMode);

    m_leftFollower.follow(m_leftLeader);
    m_rightFollower.follow(m_rightLeader);

    m_leftLeader.setInverted(driveConfig.leftSideInverted);
    m_leftFollower.setInverted(InvertType.FollowMaster);
    m_rightLeader.setInverted(driveConfig.rightSideInverted);
    m_rightFollower.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void updateInputs(DriveIOInputs inputs) {
    double leftSignum = leftSensorInverted ? -1 : 1;
    double rightSignum = rightSensorInverted ? -1 : 1;

    inputs.LeftPositionMeters = leftSignum * m_leftSensors.getPosition();
    inputs.LeftVelocityMetersPerSecond = leftSignum * m_leftSensors.getLinearVelocity();
    inputs.RightPositionMeters = rightSignum * m_rightSensors.getPosition();
    inputs.RightVelocityMetersPerSecond = rightSignum * m_rightSensors.getLinearVelocity();
  }

  @Override
  public void resetEncoders() {
    m_leftSensors.resetEncoder();
    m_rightSensors.resetEncoder();
  }

  @Override
  public void setVoltage(double leftVolts, double rightVolts) {
    leftVolts = MathUtil.clamp(leftVolts, -12.0, 12.0);
    rightVolts = MathUtil.clamp(rightVolts, -12.0, 12.0);

    m_leftLeader.setVoltage(leftVolts);
    m_rightLeader.setVoltage(rightVolts);
  }

  @Override
  public void setNeutralMode(DriveNeutralMode mode) {
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

  public record DriveIOFalconConfig(
      CANDeviceConfig leftLeader,
      CANDeviceConfig leftFollower,
      CANDeviceConfig rightLeader,
      CANDeviceConfig rightFollower,
      double gearRatio,
      double wheelRadiusMeters,
      boolean leftSideInverted,
      boolean leftSensorInverted,
      boolean rightSideInverted,
      boolean rightSensorInverted) {}
}
