package frc.robot.arm.rotation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.constants.Constants;

public class ArmRotationIONeo implements ArmRotationIO {
  private final CANSparkMax m_leader, m_follower;

  public ArmRotationIONeo(int leader, int follower) {
    this.m_leader = new CANSparkMax(leader, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.m_follower = new CANSparkMax(follower, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_leader.setIdleMode(CANSparkMax.IdleMode.kBrake);
    m_follower.setIdleMode(CANSparkMax.IdleMode.kBrake);

    m_follower.follow(m_leader);

    m_leader.setInverted(Constants.Arm.kRotationInverted);
  }

  @Override
  public void updateInputs(ArmRotationIOInputs inputs) {
    inputs.SupplyCurrentAmps = (m_leader.getAppliedOutput() + m_follower.getAppliedOutput()) / 2;
    inputs.StatorCurrentAmps = (m_leader.getOutputCurrent() + m_follower.getOutputCurrent()) / 2;
  }

  @Override
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
  }
}
