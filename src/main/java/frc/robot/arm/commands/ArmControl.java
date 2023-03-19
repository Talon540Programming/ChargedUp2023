package frc.robot.arm.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotLimits;
import frc.robot.oi.OperatorInterface;

public class ArmControl extends CommandBase {
  private final ArmBase m_armBase;
  private final OperatorInterface m_operatorInterface;

  public ArmControl(ArmBase armBase, OperatorInterface operatorInterface) {
    this.m_armBase = armBase;
    this.m_operatorInterface = operatorInterface;

    addRequirements(armBase);
  }

  @Override
  public void execute() {
    double armAngleRad = m_armBase.getTargetState().AngleRadians;
    double armLength = m_armBase.getTargetState().LengthMeters;

    if((m_operatorInterface.getRotationLinearX() != 0.0 || m_operatorInterface.getRotationLinearY() != 0.0) && !m_operatorInterface.lockRotation().getAsBoolean()) {
      // Calculate the target state from the linear axes
      armAngleRad = Math.atan2(m_operatorInterface.getRotationLinearY(), m_operatorInterface.getRotationLinearX());
      armAngleRad += armAngleRad < -Math.PI / 2.0 ? Math.PI * 2.0 : 0;
    }

    armLength += .05 * m_operatorInterface.getExtensionPercent();
    armLength = MathUtil.clamp(armLength, RobotLimits.kMinArmLengthMeters, Constants.Arm.kArmKinematics.maxArmAndEffectorLength(armAngleRad));

    System.out.println(armLength);

    m_armBase.updateState(new ArmState(armAngleRad, armLength));
  }
}
