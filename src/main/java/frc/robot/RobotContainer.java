// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.HardwareDevices;
import frc.robot.drivetrain.DrivetrainBase;
import frc.robot.drivetrain.commands.control.XboxControllerDriveControl;
import org.talon540.control.XboxController.TalonXboxController;

public class RobotContainer {
  private final TalonXboxController m_driverController = new TalonXboxController(HardwareDevices.kDriverXboxControllerPort);

  private final DrivetrainBase m_drivetrainBase = new DrivetrainBase();

  public RobotContainer() {
    configureBindings();
  }

  private void configureBindings() {
    m_drivetrainBase.setDefaultCommand(new XboxControllerDriveControl(m_drivetrainBase, m_driverController));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
