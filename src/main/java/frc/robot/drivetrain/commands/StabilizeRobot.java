package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.drivetrain.DriveBase;

public class StabilizeRobot extends CommandBase {
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
    double rawMeasurement = Math.toDegrees(m_driveBase.m_driveInputs.PitchPositionRad);
    double measurement = Math.abs(rawMeasurement) <= 3 ? 0 : rawMeasurement;

    double outputPercent = MathUtil.clamp(measurement, -0.25, 0.25);

    m_driveBase.tankDrivePercent(outputPercent, outputPercent);
  }

  @Override
  public boolean isFinished() {
    return Math.abs(Math.toDegrees(m_driveBase.m_driveInputs.PitchPositionRad))
        < Drivetrain.kRobotStabilizationToleranceDegrees;
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
