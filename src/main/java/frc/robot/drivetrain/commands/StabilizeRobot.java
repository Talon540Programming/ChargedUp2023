package frc.robot.drivetrain.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.drivetrain.DriveBase;

public class StabilizeRobot extends CommandBase {
  public static final double DrivePercent = 0.15;
  public static final double PositionThresholdRad = Math.toRadians(3.0);
  public static final double VelocityThresholdRadPerSec = Math.toRadians(8.0);

  private final DriveBase m_driveBase;

  public StabilizeRobot(DriveBase driveBase) {
    m_driveBase = driveBase;

    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    m_driveBase.setNeutralMode(Constants.NeutralMode.BRAKE);
  }

  @Override
  public void execute() {
    double gyroPitchRad = m_driveBase.m_driveInputs.PitchPositionRad;
    double gyroPitchRateRadPerSec = m_driveBase.m_driveInputs.PitchRateRadPerSecond;

    boolean shouldStop = (gyroPitchRad < 0.0 && gyroPitchRateRadPerSec > VelocityThresholdRadPerSec)
            || (gyroPitchRad > 0.0 && gyroPitchRateRadPerSec < -VelocityThresholdRadPerSec);

    if(shouldStop) {
      m_driveBase.stop();
    } else {
      double signum = gyroPitchRad > 0.0 ? -1.0 : 1.0;
      m_driveBase.tankDrivePercent(signum * DrivePercent, signum * DrivePercent);
    }
  }

  @Override
  public boolean isFinished() {
    return Math.abs(m_driveBase.m_driveInputs.PitchPositionRad) < PositionThresholdRad;
  }

  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
    if (interrupted) {
      // If the balance command is canceled, we can assume we needed to get off and brake mode
      // should be disabled.
      m_driveBase.resetNeutralMode();
    }
  }
}
