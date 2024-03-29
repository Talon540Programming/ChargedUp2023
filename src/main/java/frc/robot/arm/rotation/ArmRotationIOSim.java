package frc.robot.arm.rotation;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.system.plant.DCMotor;
import frc.robot.arm.ArmSystemDynamics;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotDimensions;

public class ArmRotationIOSim implements ArmRotationIO {
  private final VariableSingleJoinedArmSim m_armSim;

  public ArmRotationIOSim(boolean simulateGravity) {
    m_armSim =
        new VariableSingleJoinedArmSim(
            DCMotor.getNEO(2),
            Constants.Arm.kRotationGearRatio,
            ArmSystemDynamics.calculateSystemMomentOfInertia(
                RobotDimensions.Arm.kFullyRetractedLengthMeters),
            RobotDimensions.Arm.kFullyRetractedLengthMeters,
            RobotDimensions.Effector.kLengthMeters,
            Constants.Arm.kArmKinematics,
            simulateGravity);

    // Start the SIM with the arm straight up to model the actual bot
    m_armSim.setState(VecBuilder.fill(Math.PI / 2.0, 0));
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

  @Override
  public void updateArmLength(double pivotToEffectorMeters) {
    m_armSim.updateArmLength(pivotToEffectorMeters);

    m_armSim.updateMoI(ArmSystemDynamics.calculateSystemMomentOfInertia(pivotToEffectorMeters));
  }
}
