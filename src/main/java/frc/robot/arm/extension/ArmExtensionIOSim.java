package frc.robot.arm.extension;

import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotDimensions;
import frc.robot.constants.RobotLimits;

public class ArmExtensionIOSim implements ArmExtensionIO {
  private final ElevatorSim m_armSim;

  public ArmExtensionIOSim() {
    m_armSim =
        new ElevatorSim(
            DCMotor.getNEO(1),
            Constants.Arm.kExtensionGearRatio,
            RobotDimensions.Effector.kEffectorMassKg,
            Constants.Arm.kExtensionWinchRadiusMeters,
            RobotLimits.kMinArmLengthMeters,
            RobotLimits.kMaxArmLengthMeters,
            false);
  }

  @Override
  public void updateInputs(ArmExtensionIOInputs inputs) {
    m_armSim.update(Constants.loopPeriodSecs);

    inputs.CurrentAmps = m_armSim.getCurrentDrawAmps();
    inputs.DistanceTraveledMeters = m_armSim.getPositionMeters();
    inputs.VelocityMetersPerSecond = m_armSim.getVelocityMetersPerSecond();
  }

  @Override
  public void setVoltage(double voltage) {
    m_armSim.setInput(voltage);
  }
}
