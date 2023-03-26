package frc.robot.autos;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;

/**
 * Represents an Auto Routine used to drive the drivetrain at a given percent for a given duration.
 * This can be used as a failsafe auto to just get the points for taxiing.
 */
public class DriveTime extends CommandBase {
  private final double kDuration;
  private final double kSpeed;

  private final DriveBase m_driveBase;
  private final Timer m_timer = new Timer();

  /**
   * Create a DriveTime Auto.
   *
   * @param driveBase drivetrain subsystem.
   * @param duration how long to drive in seconds.
   * @param speed speed to drive at [-1.0, 1.0].
   */
  public DriveTime(DriveBase driveBase, double duration, double speed) {
    kDuration = Math.max(0, duration);
    kSpeed = MathUtil.clamp(speed, -1, 1);
    m_driveBase = driveBase;

    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_driveBase.tankDrivePercent(kSpeed, kSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(kDuration);
  }
}
