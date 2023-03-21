package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;
import frc.robot.oi.DriverInterface;

public class DriveControl extends CommandBase {
  private final DriveBase m_driveBase;
  private final DriverInterface m_driverInterface;

  public DriveControl(DriveBase driveBase, DriverInterface driverInterface) {
    this.m_driveBase = driveBase;
    this.m_driverInterface = driverInterface;

    addRequirements(driveBase);
  }

  @Override
  public void execute() {
    switch (m_driverInterface.getDriveMode()) {
      case Differential -> m_driveBase.tankDrivePercent(
          m_driverInterface.getLeftPercent(), m_driverInterface.getRightPercent());
      case Arcade -> m_driveBase.arcadeDrivePercent(
          m_driverInterface.getLeftPercent(), -m_driverInterface.getRightPercent());
    }
  }
}
