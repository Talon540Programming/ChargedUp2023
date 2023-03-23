package frc.robot.arm.extension;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.MathUtil;
import frc.lib.SparkMaxBurnManager;
import frc.lib.SparkMaxPeriodicFrameConfig;
import frc.robot.constants.Constants;

/** ArmExtensionIO using 1 SparkMax motor controller. */
public class ArmExtensionIOSparkMax implements ArmExtensionIO {
  private final CANSparkMax m_winchMotor;
  private final RelativeEncoder m_winchEncoder;

  public ArmExtensionIOSparkMax(int id, boolean motorInverted, double conversionFactor) {
    m_winchMotor = new CANSparkMax(id, CANSparkMaxLowLevel.MotorType.kBrushless);

    if (SparkMaxBurnManager.shouldBurnFlash()) m_winchMotor.restoreFactoryDefaults();

    m_winchMotor.setCANTimeout(SparkMaxBurnManager.kConfigurationStatusTimeoutMs);

    SparkMaxPeriodicFrameConfig.configureIsolated(m_winchMotor);

    m_winchMotor.setInverted(motorInverted);
    m_winchMotor.setSmartCurrentLimit(20);
    m_winchMotor.enableVoltageCompensation(12.0);

    m_winchEncoder = m_winchMotor.getEncoder();
    m_winchEncoder.setMeasurementPeriod(10);
    m_winchEncoder.setAverageDepth(2);

    m_winchEncoder.setPositionConversionFactor(conversionFactor);
    m_winchEncoder.setVelocityConversionFactor(conversionFactor / 60.0);

    m_winchMotor.setCANTimeout(0);

    if (SparkMaxBurnManager.shouldBurnFlash()) m_winchMotor.burnFlash();

    setNeutralMode(Constants.NeutralMode.BRAKE);
  }

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    inputs.PivotToEffectorDistanceMeters = m_winchEncoder.getPosition();
    inputs.VelocityMetersPerSecond = m_winchEncoder.getVelocity();
    inputs.CurrentAmps = m_winchMotor.getOutputCurrent();
    inputs.TemperatureCelsius = m_winchMotor.getMotorTemperature();
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);

    m_winchMotor.setVoltage(voltage);
  }

  @Override
  public void setDistance(double distanceMeters) {
    m_winchEncoder.setPosition(distanceMeters);
  }

  @Override
  public void resetDistance() {
    m_winchEncoder.setPosition(0);
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_winchMotor.setIdleMode(mode.toIdleMode());
  }
}
