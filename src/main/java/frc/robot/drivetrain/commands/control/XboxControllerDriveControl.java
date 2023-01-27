// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands.control;

import frc.robot.drivetrain.DriveBase;
import org.talon540.control.XboxController.TalonXboxController;

/** Command used to drive the drivetrain using an Xbox Controller. */
public class XboxControllerDriveControl extends DriveControl {
  private final TalonXboxController controller;

  /**
   * Construct a command that can be used to control the drivetrain using an XboxController as an
   * {@link TalonXboxController}.
   *
   * @param driveBase Drivetrain subsystem.
   * @param controller Driver XboxController.
   */
  public XboxControllerDriveControl(DriveBase driveBase, TalonXboxController controller) {
    super(driveBase);
    this.controller = controller;
  }

  @Override
  public void execute() {
    this.kLeftPercent = controller.getLeftDeadbandY();
    this.kRightPercent = controller.getRightDeadbandY();
    super.execute();
  }
}
