package frc.robot.arm.rotation;

import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.math.MathUtil;
import frc.robot.constants.Constants;

/** ArmRotationIO using 2 SparkMax motor controllers. */
public class ArmRotationIOSparkMax implements ArmRotationIO {
  private final CANSparkMax m_rotationLeader, m_rotationFollower;
  private final WPI_CANCoder m_absoluteEncoder;

  /**
   * Create an IO layer for controlling two SparkMaxes for ArmRotation.
   *
   * @param rotationLeader id of leader SparkMax.
   * @param rotationFollower id of the follower SparkMax
   * @param rotationInverted whether the direction of the motors should be inverted.
   * @param encoderID id of the CANcoder
   * @param absoluteEncoderOffset offset of the CANCoder in degrees.
   */
  public ArmRotationIOSparkMax(
      int rotationLeader,
      int rotationFollower,
      boolean rotationInverted,
      int encoderID,
      double absoluteEncoderOffset) {
    // CONFIGURE ENCODER
    this.m_absoluteEncoder = new WPI_CANCoder(encoderID);

    CANCoderConfiguration config = new CANCoderConfiguration();
    config.sensorCoefficient = 2 * Math.PI / 4096.0;
    config.unitString = "rad";
    config.magnetOffsetDegrees = absoluteEncoderOffset;
    config.absoluteSensorRange = AbsoluteSensorRange.Unsigned_0_to_360;

    this.m_absoluteEncoder.configAllSettings(config);

    // CONFIGURE ROTATION MOTORS
    this.m_rotationLeader =
        new CANSparkMax(rotationLeader, CANSparkMaxLowLevel.MotorType.kBrushless);
    this.m_rotationFollower =
        new CANSparkMax(rotationFollower, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_rotationFollower.follow(m_rotationLeader);
    m_rotationLeader.setInverted(rotationInverted);

    m_rotationLeader.setSmartCurrentLimit(30);
    m_rotationFollower.setSmartCurrentLimit(30);

    m_rotationLeader.enableVoltageCompensation(12.0);

    m_rotationLeader.setCANTimeout(0);
    m_rotationFollower.setCANTimeout(0);

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
    m_rotationFollower.setIdleMode(mode.toIdleMode());
    m_rotationFollower.setIdleMode(mode.toIdleMode());
  }
}
