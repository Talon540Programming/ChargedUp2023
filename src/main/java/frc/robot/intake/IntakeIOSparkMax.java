package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import frc.lib.SparkMaxBurnManager;
import frc.lib.SparkMaxPeriodicFrameConfig;
import frc.robot.constants.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax m_leader;
  private final CANSparkMax m_follower;

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  public IntakeIOSparkMax(int leftId, int rightId, double conversionFactor) {
    m_leader = new CANSparkMax(leftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_follower = new CANSparkMax(rightId, CANSparkMaxLowLevel.MotorType.kBrushless);

    if (SparkMaxBurnManager.shouldBurnFlash()) {
      m_leader.restoreFactoryDefaults();
      m_follower.restoreFactoryDefaults();
    }

    m_leader.setCANTimeout(SparkMaxBurnManager.kConfigurationStatusTimeoutMs);
    m_follower.setCANTimeout(SparkMaxBurnManager.kConfigurationStatusTimeoutMs);

    SparkMaxPeriodicFrameConfig.configureLeader(m_leader);
    SparkMaxPeriodicFrameConfig.configureFollower(m_follower);

    m_leader.setInverted(true);
    m_follower.follow(m_leader);

    m_leader.setSmartCurrentLimit(20);
    m_follower.setSmartCurrentLimit(20);

    m_leader.enableVoltageCompensation(12.0);

    m_leftEncoder = m_leader.getEncoder();
    m_leftEncoder.setMeasurementPeriod(10);
    m_leftEncoder.setAverageDepth(2);
    m_leftEncoder.setPositionConversionFactor(conversionFactor);
    m_leftEncoder.setVelocityConversionFactor(conversionFactor / 60.0);

    m_rightEncoder = m_follower.getEncoder();
    m_rightEncoder.setMeasurementPeriod(10);
    m_rightEncoder.setAverageDepth(2);
    m_rightEncoder.setPositionConversionFactor(conversionFactor);
    m_rightEncoder.setVelocityConversionFactor(conversionFactor / 60.0);

    m_leader.setCANTimeout(0);
    m_follower.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurnFlash()) {
      m_leader.burnFlash();
      m_follower.burnFlash();
    }

    setNeutralMode(Constants.NeutralMode.BRAKE);
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.CurrentAmps = new double[] {m_leader.getOutputCurrent(), m_follower.getOutputCurrent()};
    inputs.TemperatureCelsius =
        new double[] {m_leader.getMotorTemperature(), m_follower.getMotorTemperature()};

    inputs.LeftVelocityRadPerSecond = m_leftEncoder.getVelocity();
    inputs.RightVelocityRadPerSecond = m_rightEncoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    m_leader.setVoltage(voltage);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_leader.setIdleMode(mode.toIdleMode());
    m_follower.setIdleMode(mode.toIdleMode());
  }

  @Override
  public boolean isStalled() {
    return m_leader.getOutputCurrent() >= 20;
  }
}
