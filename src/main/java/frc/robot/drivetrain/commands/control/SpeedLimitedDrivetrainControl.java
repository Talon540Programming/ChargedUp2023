package frc.robot.drivetrain.commands.control;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants.Drivetrain;
import frc.robot.drivetrain.DrivetrainBase;

public abstract class SpeedLimitedDrivetrainControl extends CommandBase {
  protected double kLinearPercent, kRotationalPercent;
  private final DrivetrainBase drivetrainBase;

  public SpeedLimitedDrivetrainControl(DrivetrainBase drivetrainBase) {
    addRequirements(drivetrainBase);
    this.drivetrainBase = drivetrainBase;
  }

  @Override
  public void execute() {
    this.drivetrainBase.setFromForces(
        Drivetrain.kMaxDrivetrainVelocityMetersPerSecond * kLinearPercent,
        Drivetrain.kMaxDrivetrainRotationalVelocityRadiansPerSecond * kRotationalPercent);
  }
}
