package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drivetrain.commands.DriveControl;

/** Represents the inputs required for the primary driver. */
public interface DriverInterface {
  public double getLeftPercent(DriveControl.DriveMode mode);

  public double getRightPercent(DriveControl.DriveMode mode);

  public Trigger toggleBalanceMode();

  public Trigger enableBrakeMode();

  public Trigger enableCoastMode();

}
