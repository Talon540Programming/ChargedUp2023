package frc.robot.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotLimits;
import frc.robot.oi.DriverInterface;
import frc.robot.oi.OperatorInterface;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ArmControl extends CommandBase {
  private final ArmBase m_armBase;
  private final OperatorInterface m_operatorInterface;

  private final LoggedDashboardChooser<OperatorInterface.ArmMode> m_armMode =
          new LoggedDashboardChooser<>("Arm Mode");

  public ArmControl(ArmBase armBase, OperatorInterface operatorInterface) {
    this.m_armBase = armBase;
    this.m_operatorInterface = operatorInterface;

    m_armMode.addDefaultOption("State Space", OperatorInterface.ArmMode.StateSpace);
    m_armMode.addOption("Direct (Voltage Applied)", OperatorInterface.ArmMode.Direct);

    addRequirements(armBase);
  }

  @Override
  public void execute() {
    switch (m_armMode.get()) {
      case StateSpace -> {
        double armAngleRad = m_armBase.getTargetState().AngleRadians;
        double armLength = m_armBase.getTargetState().LengthMeters;

        if ((m_operatorInterface.getRotationLinearX() != 0.0
                || m_operatorInterface.getRotationLinearY() != 0.0)
                && !m_operatorInterface.lockRotation().getAsBoolean()) {
          // Calculate the target state from the linear axes
          armAngleRad =
                  Math.atan2(
                          m_operatorInterface.getRotationLinearY(), m_operatorInterface.getRotationLinearX());
          armAngleRad += armAngleRad < -Math.PI / 2.0 ? Math.PI * 2.0 : 0;
        }

        // Extend at a proportional rate of 5cm per 20ms * extension percent [-1, 1] (given ideal loop time)
        armLength += .05 * m_operatorInterface.getExtensionPercent();
        armLength =
                MathUtil.clamp(
                        armLength,
                        RobotLimits.kMinArmLengthMeters,
                        Constants.Arm.kArmKinematics.maxArmAndEffectorLength(armAngleRad));

        m_armBase.updateState(new ArmState(armAngleRad, armLength));
      }
      case Direct -> {
        m_armBase.setExtensionVoltage(m_operatorInterface.getExtensionPercent() * 12.0);
        m_armBase.setRotationVoltage(m_operatorInterface.getRotationLinearY() * 12.0);
      }
    }
  }
}
