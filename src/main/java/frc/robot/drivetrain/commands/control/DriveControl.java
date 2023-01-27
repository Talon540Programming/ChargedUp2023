// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.drivetrain.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public abstract class DriveControl extends CommandBase {
  protected double kLeftPercent, kRightPercent;
  private final DriveBase m_driveBase;

  private static final LoggedDashboardChooser<Double> m_speedLimiter =
      new LoggedDashboardChooser<>("Drive Speed Limit");

  static {
    m_speedLimiter.addDefaultOption("Default", 1.0);
    m_speedLimiter.addOption("Fast (70%)", 0.7);
    m_speedLimiter.addOption("Medium (39%)", 0.3);
    m_speedLimiter.addOption("Slow (15%)", 0.15);
  }

  protected DriveControl(DriveBase driveBase) {
    this.m_driveBase = driveBase;

    addRequirements(driveBase);
  }

  @Override
  public void execute() {
    kLeftPercent *= m_speedLimiter.get();
    kRightPercent *= m_speedLimiter.get();

    this.m_driveBase.tankDrivePercent(kLeftPercent, kRightPercent);
  }
}
