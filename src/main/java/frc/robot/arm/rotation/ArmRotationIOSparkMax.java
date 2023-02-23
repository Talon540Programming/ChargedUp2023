package frc.robot.arm.rotation;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import frc.robot.constants.Constants;

/** ArmRotationIO using 2 SparkMax motor controllers. */
public class ArmRotationIOSparkMax implements ArmRotationIO {
  private final CANSparkMax m_leader, m_follower;

  /**
   * Create an IO layer for controlling two SparkMaxes for ArmRotation.
   *
   * @param leader id of leader SparkMax.
   * @param follower id of the follower SparkMax
   * @param inverted whether the direction of the motors should be inverted.
   */
  public ArmRotationIOSparkMax(int leader, int follower, boolean inverted) {
    this.m_leader = new CANSparkMax(leader, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.m_follower = new CANSparkMax(follower, CANSparkMaxLowLevel.MotorType.kBrushless);

    this.m_leader.setSmartCurrentLimit(50);
    this.m_follower.setSmartCurrentLimit(50);

    setNeutralMode(Constants.NeutralMode.BRAKE);

    m_follower.follow(m_leader);
    m_leader.setInverted(inverted);
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

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    switch (mode) {
      case BRAKE -> {
        m_leader.setIdleMode(CANSparkMax.IdleMode.kBrake);
        m_follower.setIdleMode(CANSparkMax.IdleMode.kBrake);
      }
      case COAST -> {
        m_leader.setIdleMode(CANSparkMax.IdleMode.kCoast);
        m_follower.setIdleMode(CANSparkMax.IdleMode.kCoast);
      }
    }
  }
}
