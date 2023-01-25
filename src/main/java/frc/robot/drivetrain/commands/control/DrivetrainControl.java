package frc.robot.drivetrain.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;

public abstract class DrivetrainControl extends CommandBase {
  protected double kLeftPercent, kRightPercent;
  private final DriveBase m_driveBase;

  protected DrivetrainControl(DriveBase driveBase) {
    this.m_driveBase = driveBase;

    addRequirements(driveBase);
  }

  @Override
  public void execute() {
    this.m_driveBase.tankDrivePercent(kLeftPercent, kRightPercent);
  }
}
