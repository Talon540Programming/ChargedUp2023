package frc.robot.drivetrain.commands.control;

import frc.robot.drivetrain.DrivetrainBase;
import org.talon540.control.XboxController.TalonXboxController;

/** Command used to drive the drivetrain using an Xbox Controller. */
public class XboxControllerDriveControl extends DrivetrainControl {
  private final TalonXboxController controller;

  /**
   * Construct a command that can be used to control the drivetrain using an XboxController as an
   * {@link TalonXboxController}.
   *
   * @param drivetrainBase Drivetrain subsystem.
   * @param controller Driver XboxController.
   */
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
