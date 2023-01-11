package frc.robot.arm.extension.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.extension.ArmExtensionBase;
import frc.robot.arm.extension.ArmExtensionBase.ArmExtensionPosition;
import frc.robot.constants.Constants;

public class ExtendToPosition extends CommandBase {
  private final ArmExtensionBase extensionBase;
  private final ArmExtensionPosition targetPosition;

  public ExtendToPosition(ArmExtensionBase armExtensionBase, ArmExtensionPosition targetPosition) {
    addRequirements(armExtensionBase);

    this.extensionBase = armExtensionBase;
    if (targetPosition == ArmExtensionPosition.kUnknown)
      throw new IllegalArgumentException("Cannot extend to an unknown position");
    this.targetPosition = targetPosition;
  }

  @Override
  public void execute() {
    ArmExtensionBase.ArmExtensionPosition lastKnownArmPosition = extensionBase.getLastPosition();

    if (lastKnownArmPosition == ArmExtensionPosition.kUnknown) {
      // The position of the arm is unknown, move forward until it is known, hopefully before it
      // hits the absolute max.
      extensionBase.setExtensionPercent(0.5);
      return;
    }

    // Get the position of the target relative to the last position
    extensionBase.setExtensionPercent(Math.signum(targetPosition.compareTo(lastKnownArmPosition)) * Constants.kExtensionRate);
  }

  @Override
  public boolean isFinished() {
    return extensionBase.getCurrentPosition() == targetPosition;
  }

  @Override
  public void end(boolean interrupted) {
    extensionBase.stopExtension();
  }
}
