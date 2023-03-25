package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;

public class DriveDistance extends CommandBase {
  private final DriveBase m_driveBase;

  private final double driveDistanceMeters;

  private double initialLeftDistanceMeters;
  private double initialRightDistanceMeters;

  public DriveDistance(double distanceMeters, DriveBase driveBase) {
    this.m_driveBase = driveBase;
    this.driveDistanceMeters = distanceMeters;

    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    initialLeftDistanceMeters = m_driveBase.m_driveInputs.LeftPositionMeters;
    initialRightDistanceMeters = m_driveBase.m_driveInputs.RightPositionMeters;
  }

  @Override
  public void execute() {
    double distanceTraveledLeft =
        m_driveBase.m_driveInputs.LeftPositionMeters - initialLeftDistanceMeters;
    // double distanceTraveledRight = m_driveBase.m_driveInputs.RightPositionMeters -
    // initialRightDistanceMeters;

    double deltaLeftDistance = driveDistanceMeters - distanceTraveledLeft;
    // double deltaRightDistance = driveDistanceMeters - distanceTraveledRight;

    double speed = MathUtil.clamp(deltaLeftDistance, -0.5, 0.5);

    m_driveBase.tankDrivePercent(speed, speed);
  }

  @Override
  public boolean isFinished() {
    double distanceTraveledLeft =
        m_driveBase.m_driveInputs.LeftPositionMeters - initialLeftDistanceMeters;

    return Math.abs(driveDistanceMeters - distanceTraveledLeft) <= 0.1;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
  }
}
