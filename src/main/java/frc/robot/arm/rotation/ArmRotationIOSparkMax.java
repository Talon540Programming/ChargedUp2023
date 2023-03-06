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

    m_follower.follow(m_leader);
    m_leader.setInverted(inverted);

    m_leader.setSmartCurrentLimit(40);
    m_follower.setSmartCurrentLimit(40);

    m_leader.enableVoltageCompensation(12.0);

    m_leader.setCANTimeout(0);
    m_follower.setCANTimeout(0);

    setNeutralMode(Constants.NeutralMode.BRAKE);
  }

  @Override
  public void updateInputs(ArmRotationIOInputs inputs) {
    inputs.CurrentAmps = new double[] {m_leader.getOutputCurrent(), m_follower.getOutputCurrent()};
    inputs.TemperatureCelsius =
        new double[] {m_leader.getMotorTemperature(), m_follower.getMotorTemperature()};
  }

  @Override
  public void setVoltage(double voltage) {
    m_leader.setVoltage(voltage);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_leader.setIdleMode(mode.toIdleMode());
    m_follower.setIdleMode(mode.toIdleMode());
  }
}
