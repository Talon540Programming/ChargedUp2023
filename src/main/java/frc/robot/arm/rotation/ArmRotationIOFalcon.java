package frc.robot.arm.rotation;

import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import frc.robot.constants.Constants;
import org.talon540.hardware.CANDeviceConfig;

public class ArmRotationIOFalcon implements ArmRotationIO {
  private final WPI_TalonFX m_leader, m_follower;

  public ArmRotationIOFalcon(CANDeviceConfig leader, CANDeviceConfig follower) {
    this.m_leader = new WPI_TalonFX(leader.id, leader.controller);
    this.m_follower = new WPI_TalonFX(follower.id, follower.controller);

    TalonFXConfiguration config = new TalonFXConfiguration();
    config.voltageCompSaturation = 12.0;
    config.statorCurrLimit = new StatorCurrentLimitConfiguration(true, 40, 0, 0);

    m_leader.configAllSettings(config);
    m_follower.configAllSettings(config);

    setNeutralMode(Constants.NeutralMode.BRAKE);

    m_follower.follow(m_leader);

    m_follower.setInverted(Constants.Arm.kRotationInverted);
    m_leader.setInverted(InvertType.FollowMaster);
  }

  @Override
  public void updateInputs(ArmRotationIOInputs inputs) {
    inputs.SupplyCurrentAmps = (m_leader.getSupplyCurrent() + m_follower.getSupplyCurrent()) / 2;
    inputs.StatorCurrentAmps = (m_leader.getStatorCurrent() + m_follower.getStatorCurrent()) / 2;
  }

  @Override
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    switch (mode) {
      case BRAKE -> {
        m_leader.setNeutralMode(NeutralMode.Brake);
        m_follower.setNeutralMode(NeutralMode.Brake);
      }
      case COAST -> {
        m_leader.setNeutralMode(NeutralMode.Coast);
        m_follower.setNeutralMode(NeutralMode.Coast);
      }
    }
  }
}
