package frc.robot.drivetrain.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DrivetrainBase;

public abstract class DrivetrainControl extends CommandBase {
  protected double kLeftPercent, kRightPercent;
  private final DrivetrainBase m_drivetrainBase;

  public DrivetrainControl(DrivetrainBase drivetrainBase) {
    addRequirements(drivetrainBase);
    this.m_drivetrainBase = drivetrainBase;
  }

  @Override
  public void execute() {
    this.m_drivetrainBase.tankDrivePercent(kLeftPercent, kRightPercent);
  }
}
