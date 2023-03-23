package frc.robot.intake;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.constants.Constants;

public class IntakeIOSim implements IntakeIO {
  private final DCMotor m_leftMotor;
  private final DCMotor m_rightMotor;

  private double leftVoltage;
  private double rightVoltage;

  public IntakeIOSim() {
    m_leftMotor = DCMotor.getNeo550(1).withReduction(Constants.Intake.kGearRatio);
    m_rightMotor = DCMotor.getNeo550(1).withReduction(Constants.Intake.kGearRatio);
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);

    leftVoltage = voltage;
    rightVoltage = -voltage;
  }

  @Override
  public void updateInputs(IntakeInputs inputs) {
    inputs.LeftVelocityRadPerSecond = m_leftMotor.KvRadPerSecPerVolt * leftVoltage;
    inputs.RightVelocityRadPerSecond = m_rightMotor.KvRadPerSecPerVolt * rightVoltage;

    inputs.CurrentAmps =
        new double[] {
          m_leftMotor.getCurrent(inputs.LeftVelocityRadPerSecond, leftVoltage),
          m_rightMotor.getCurrent(inputs.RightVelocityRadPerSecond, rightVoltage)
        };
  }
}
