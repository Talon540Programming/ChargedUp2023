package frc.robot.arm.rotation;

import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
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
  public ArmRotationIOSparkMax(
      int rotationLeader,
      int rotationFollower,
      boolean rotationInverted,
      int encoderID,
      double absoluteEncoderOffset) {
    // CONFIGURE ENCODER
    this.m_absoluteEncoder = new WPI_CANCoder(encoderID);

    m_follower.follow(m_leader);
    m_leader.setInverted(inverted);

    this.m_absoluteEncoder.configAllSettings(config);

    // CONFIGURE ROTATION MOTORS
    this.m_rotationLeader =
        new CANSparkMax(rotationLeader, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.m_rotationFollower =
        new CANSparkMax(rotationFollower, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_leader.setCANTimeout(0);
    m_follower.setCANTimeout(0);

    setNeutralMode(Constants.NeutralMode.BRAKE);
  }

  @Override
  public void updateInputs(ArmRotationIOInputs inputs) {
    inputs.CurrentAmps =
        new double[] {m_rotationLeader.getOutputCurrent(), m_rotationFollower.getOutputCurrent()};
    inputs.TemperatureCelsius =
        new double[] {
          m_rotationLeader.getMotorTemperature(), m_rotationFollower.getMotorTemperature()
        };
    inputs.ArmVelocityRadPerSecond = m_absoluteEncoder.getAbsolutePosition();
    inputs.AbsoluteArmPositionRad = m_absoluteEncoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);

    m_rotationLeader.setVoltage(voltage);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_leader.setIdleMode(mode.toIdleMode());
    m_follower.setIdleMode(mode.toIdleMode());
  }
}
