package frc.robot.drivetrain.commands.control;

import frc.robot.drivetrain.DrivetrainBase;
import org.talon540.control.XboxController.TalonXboxController;

public class XboxControllerDriveControl extends DrivetrainControl {
  private final TalonXboxController controller;

  public XboxControllerDriveControl(DrivetrainBase drivetrainBase, TalonXboxController controller) {
    super(drivetrainBase);
    this.controller = controller;
  }

  @Override
  public void execute() {
    this.kLeftPercent = controller.getLeftDeadbandY();
    this.kRightPercent = controller.getRightDeadbandY();
    super.execute();
  }
}
