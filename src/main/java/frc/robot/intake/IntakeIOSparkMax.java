package frc.robot.intake;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.ColorSensorV3;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.Constants;

public class IntakeIOSparkMax implements IntakeIO {
  private final CANSparkMax m_leftMotor;
  private final CANSparkMax m_rightMotor;

  private final RelativeEncoder m_leftEncoder;
  private final RelativeEncoder m_rightEncoder;

  private final ColorSensorV3 m_colorSensor;

  public IntakeIOSparkMax(
      int leftId,
      int rightId,
      I2C.Port colorSensorPort,
      double conversionFactor) {
    m_leftMotor = new CANSparkMax(leftId, CANSparkMaxLowLevel.MotorType.kBrushless);
    m_rightMotor = new CANSparkMax(rightId, CANSparkMaxLowLevel.MotorType.kBrushless);

    m_rightMotor.follow(m_leftMotor, true);

    m_leftMotor.setSmartCurrentLimit(20);
    m_rightMotor.setSmartCurrentLimit(20);

    m_leftMotor.enableVoltageCompensation(12.0);

    m_leftEncoder = m_leftMotor.getEncoder();
    m_leftEncoder.setMeasurementPeriod(10);
    m_leftEncoder.setAverageDepth(2);
    m_leftEncoder.setPositionConversionFactor(conversionFactor);
    m_leftEncoder.setVelocityConversionFactor(conversionFactor / 60.0);

    m_rightEncoder = m_rightMotor.getEncoder();
    m_rightEncoder.setMeasurementPeriod(10);
    m_rightEncoder.setAverageDepth(2);
    m_rightEncoder.setPositionConversionFactor(conversionFactor);
    m_rightEncoder.setVelocityConversionFactor(conversionFactor / 60.0);

    m_leftMotor.setCANTimeout(0);
    m_rightMotor.setCANTimeout(0);

    setNeutralMode(Constants.NeutralMode.BRAKE);

    m_colorSensor = new ColorSensorV3(colorSensorPort);

    if (!m_colorSensor.isConnected()) {
      DriverStation.reportError(
          "Unable to Communicate with the Color Sensor, please make sure it is plugged into the correct I2C port and is powered.",
          false);
    }
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.InfraredValue = m_colorSensor.getIR();
    inputs.RedValue = m_colorSensor.getRed();
    inputs.BlueValue = m_colorSensor.getBlue();
    inputs.GreenValue = m_colorSensor.getGreen();
    inputs.ProximityValueCm = m_colorSensor.getProximity() / 2047.0 * 10.0; // [0, 10] cm range

    inputs.CurrentAmps =
        new double[] {m_leftMotor.getOutputCurrent(), m_rightMotor.getOutputCurrent()};
    inputs.TemperatureCelsius =
        new double[] {m_leftMotor.getMotorTemperature(), m_rightMotor.getMotorTemperature()};

    inputs.LeftVelocityRadPerSecond = m_leftEncoder.getVelocity();
    inputs.RightVelocityRadPerSecond = m_rightEncoder.getVelocity();
  }

  @Override
  public void setVoltage(double voltage) {
    m_leftMotor.setVoltage(voltage);
  }

  @Override
  public Color8Bit getColor8Bit() {
    return new Color8Bit(m_colorSensor.getColor());
  }

  @Override
  public void setNeutralMode(Constants.NeutralMode mode) {
    m_leftMotor.setIdleMode(mode.toIdleMode());
    m_rightMotor.setIdleMode(mode.toIdleMode());
  }
}
