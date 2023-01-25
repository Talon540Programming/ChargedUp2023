package frc.robot.drivetrain.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.constants.Flags;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.DriveIO;

public class StabilizeRobot extends CommandBase {
  private final DriveBase m_driveBase;

  private final PIDController m_stabilizationController =
      new PIDController(
          Drivetrain.ControlValues.Stabilization.kP,
          Drivetrain.ControlValues.Stabilization.kI,
          Drivetrain.ControlValues.Stabilization.kD);

  public StabilizeRobot(DriveBase driveBase) {
    m_driveBase = driveBase;

    // We want a robot pitch of 0.
    m_stabilizationController.setSetpoint(0.0);
    m_stabilizationController.setTolerance(Drivetrain.kRobotStabilizationToleranceDegrees);

    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    m_stabilizationController.reset();
    m_driveBase.setNeutralMode(DriveIO.DriveNeutralMode.BRAKE);
  }

  @Override
  public void execute() {
    double measurement = Math.toDegrees(m_driveBase.m_gyro.getPitch());
    double outputPercent = -m_stabilizationController.calculate(measurement);

    m_driveBase.tankDrivePercent(outputPercent, outputPercent);
  }

  @Override
  public boolean isFinished() {
    return m_stabilizationController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_driveBase.resetNeutralMode();
    m_driveBase.stop();
  }
}
