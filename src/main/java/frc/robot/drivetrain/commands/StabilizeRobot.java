package frc.robot.drivetrain.commands;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.drivetrain.DrivetrainBase;

public class StabilizeRobot extends CommandBase {
  private final DrivetrainBase m_drivetrainBase;

  private final PIDController m_stabilizationController =
      new PIDController(
          Drivetrain.ControlValues.Stabilization.kP,
          Drivetrain.ControlValues.Stabilization.kI,
          Drivetrain.ControlValues.Stabilization.kD);

  public StabilizeRobot(DrivetrainBase drivetrainBase) {
    m_drivetrainBase = drivetrainBase;

    // We want a robot pitch of 0;
    m_stabilizationController.setSetpoint(0.0);
    m_stabilizationController.setTolerance(Drivetrain.kRobotStabilizationTolerance);

    addRequirements(drivetrainBase);
  }

  @Override
  public void initialize() {
    m_stabilizationController.reset();
    m_drivetrainBase.setNeutralMode(NeutralMode.Brake);
  }

  @Override
  public void execute() {
    double measurement = m_drivetrainBase.m_gyro.getPitch();
    double outputPercent = -m_stabilizationController.calculate(measurement);

    m_drivetrainBase.tankDrivePercent(
        outputPercent, outputPercent); // TODO swap this to closed loop control?
  }

  @Override
  public boolean isFinished() {
    return m_stabilizationController.atSetpoint();
  }

  @Override
  public void end(boolean interrupted) {
    m_drivetrainBase.resetNeutralMode();
    m_drivetrainBase.stop();
  }
}
