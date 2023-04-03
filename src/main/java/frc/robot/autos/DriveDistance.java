package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;

public class DriveDistance extends CommandBase {
  private final DriveBase m_driveBase;

  private final double driveDistanceMeters;

  private double targetDistanceMeters;

  public DriveDistance(DriveBase driveBase, double distanceMeters) {
    this.m_driveBase = driveBase;
    this.driveDistanceMeters = distanceMeters;

    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    targetDistanceMeters = m_driveBase.m_driveInputs.LeftPositionMeters + driveDistanceMeters;
  }

  @Override
  public void execute() {
    double deltaDistance = targetDistanceMeters - m_driveBase.m_driveInputs.LeftPositionMeters;
    double speed = MathUtil.clamp(deltaDistance, -0.5, 0.5);

    m_driveBase.tankDrivePercent(speed, speed);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_driveBase.m_driveInputs.LeftPositionMeters - targetDistanceMeters) <= 0.1;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
  }
}
