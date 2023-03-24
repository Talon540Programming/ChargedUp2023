package frc.robot.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotLimits;
import frc.robot.oi.OperatorInterface;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class ArmControl extends CommandBase {
  private final ArmBase m_armBase;
  private final OperatorInterface m_operatorInterface;

  private final LoggedDashboardChooser<OperatorInterface.ArmMode> m_armMode =
      new LoggedDashboardChooser<>("Arm Mode");

  private final LoggedDashboardChooser<Double> m_rotationSpeedLimiter =
      new LoggedDashboardChooser<>("Arm Rotation Limiter");

  private final LoggedDashboardChooser<Double> m_extensionSpeedLimiter =
      new LoggedDashboardChooser<>("Arm Extension Limiter");

  public ArmControl(ArmBase armBase, OperatorInterface operatorInterface) {
    this.m_armBase = armBase;
    this.m_operatorInterface = operatorInterface;

    m_armMode.addDefaultOption("State Space", OperatorInterface.ArmMode.StateSpace);
    m_armMode.addOption("Direct (Voltage Applied)", OperatorInterface.ArmMode.Direct);

    m_rotationSpeedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_rotationSpeedLimiter.addOption("Fast (70%)", 0.7);
    m_rotationSpeedLimiter.addOption("Medium (30%)", 0.3);
    m_rotationSpeedLimiter.addOption("Slow (15%)", 0.15);

    m_extensionSpeedLimiter.addDefaultOption("Default (100%)", 1.0);
    m_extensionSpeedLimiter.addOption("Fast (70%)", 0.7);
    m_extensionSpeedLimiter.addOption("Medium (30%)", 0.3);
    m_extensionSpeedLimiter.addOption("Slow (15%)", 0.15);

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
                  m_operatorInterface.getRotationLinearY(),
                  m_operatorInterface.getRotationLinearX());
          armAngleRad += armAngleRad < -Math.PI / 2.0 ? Math.PI * 2.0 : 0;
        }

        // Extend at a proportional rate of 5cm per 20ms * extension percent [-1, 1] (given ideal
        // loop time)
        armLength += .05 * m_operatorInterface.getExtensionPercent();
        armLength =
            MathUtil.clamp(
                armLength,
                RobotLimits.kMinArmLengthMeters,
                Constants.Arm.kArmKinematics.maxArmAndEffectorLength(armAngleRad));

        m_armBase.updateState(new ArmState(armAngleRad, armLength));
      }
      case Direct -> {
        m_armBase.setRotationVoltage(
            m_operatorInterface.getRotationLinearY() * 12.0 * m_rotationSpeedLimiter.get());
        m_armBase.setExtensionVoltage(
            m_operatorInterface.getExtensionPercent() * 12.0 * m_extensionSpeedLimiter.get());
      }
    }
  }
}
