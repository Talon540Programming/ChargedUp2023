package frc.robot.arm.commands.control;

import frc.robot.arm.ArmBase;
import org.talon540.control.XboxController.TalonXboxController;

public class XboxControllerArmControl extends ArmControl {
  private final TalonXboxController controller;

  public XboxControllerArmControl(ArmBase armBase, TalonXboxController controller) {
    super(armBase);
    this.controller = controller;
  }

  @Override
  public void execute() {
    this.kExtensionPercent = controller.getLeftDeadbandY();
    this.kRotationPercent = controller.getRightDeadbandY();
    super.execute();
  }
}
