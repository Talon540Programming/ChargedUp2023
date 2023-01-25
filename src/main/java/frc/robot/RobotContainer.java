// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.constants.Constants;
import frc.robot.constants.HardwareDevices;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.DriveIO;
import frc.robot.drivetrain.DriveIOFalcon;
import frc.robot.drivetrain.commands.control.XboxControllerDriveControl;
import frc.robot.drivetrain.gyro.GyroIO;
import frc.robot.drivetrain.gyro.GyroIOPigeon;
import org.talon540.control.XboxController.TalonXboxController;

public class RobotContainer {
  // Subsystems
  private final DriveBase m_driveBase;

  // Controllers
  private final TalonXboxController m_driverController =
      new TalonXboxController(HardwareDevices.kDriverXboxControllerPort);
  private final TalonXboxController m_depositionController =
      new TalonXboxController(HardwareDevices.kDepositionXboxControllerPort);

  public RobotContainer() {
    DriveIO driveIO;
    GyroIO gyroIO;

    switch (Constants.kCurrentMode) {
      case COMP, PROTO -> {
        driveIO = new DriveIOFalcon();
        gyroIO = new GyroIOPigeon();
      }
      default -> {
        driveIO = new DriveIO() {};
        gyroIO = new GyroIO() {};
      }
    }

    m_driveBase = new DriveBase(driveIO, gyroIO);

    configureBindings();
  }

  private void configureBindings() {
    m_driveBase.setDefaultCommand(new XboxControllerDriveControl(m_driveBase, m_driverController));
  }

  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }
}
