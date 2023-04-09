package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.drivetrain.commands.DriveControl;

/** Represents the inputs required for the primary driver. */
public interface DriverInterface {
  double getLeftPercent(DriveControl.DriveMode mode);

  double getRightPercent(DriveControl.DriveMode mode);

  Trigger toggleBalanceMode();

  Trigger enableBrakeMode();

  Trigger enableCoastMode();
}
