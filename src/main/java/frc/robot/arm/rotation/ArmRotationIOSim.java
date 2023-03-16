package frc.robot.arm.rotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotDimensions;
import frc.robot.constants.RobotLimits;

public class ArmRotationIOSim implements ArmRotationIO {
  private final SingleJointedArmSim m_armSim;

  public ArmRotationIOSim(boolean simulateGravity) {
    double effectiveInitialArmLengthMeters =
        RobotDimensions.Arm.kFullyRetractedLengthMeters + RobotDimensions.Effector.kLengthMeters;

    this.m_armSim =
        new SingleJointedArmSim(
            DCMotor.getNEO(2),
            Constants.Arm.kRotationGearRatio,
            SingleJointedArmSim.estimateMOI(
                effectiveInitialArmLengthMeters, RobotDimensions.kArmAndEffectorWeightKg),
            effectiveInitialArmLengthMeters,
            RobotLimits.kMinArmAngleRadians,
            RobotLimits.kMaxArmAngleRadians,
            simulateGravity);
  }

  @Override
  public void updateInputs(ArmRotationIOInputs inputs) {
    m_armSim.update(Constants.loopPeriodSecs);

    inputs.AbsoluteArmPositionRad = m_armSim.getAngleRads();
    inputs.ArmVelocityRadPerSecond = m_armSim.getVelocityRadPerSec();
    inputs.CurrentAmps = new double[] {m_armSim.getCurrentDrawAmps()};
  }

  @Override
  public void setVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12.0, 12.0);

    m_armSim.setInputVoltage(voltage);
  }
}
