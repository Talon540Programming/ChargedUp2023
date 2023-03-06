package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;

public class DriveDistance extends CommandBase {
  private final DriveBase m_driveBase;
  private final double distance;

  public DriveDistance(double distanceMeters, DriveBase driveBase) {
    this.m_driveBase = driveBase;
    this.distance = m_driveBase.getLinearDistanceTraveled() + distanceMeters;
    addRequirements(driveBase);
  }

  @Override
  public void execute() {
    double deltaDistance = distance - m_driveBase.getLinearDistanceTraveled();
    double speed = MathUtil.clamp(deltaDistance, -0.5, 0.5);

    m_driveBase.tankDrivePercent(speed, speed);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_driveBase.getLinearDistanceTraveled() - distance) <= 0.1;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
  }
}
