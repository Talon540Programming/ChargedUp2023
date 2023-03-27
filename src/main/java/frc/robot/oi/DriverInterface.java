package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Represents the inputs required for the primary driver. */
public interface DriverInterface {
  public double getLeftPercent(DriveMode mode);

  public double getRightPercent(DriveMode mode);

  public Trigger toggleBalanceMode();

  public Trigger enableBrakeMode();

  public Trigger enableCoastMode();

  public enum DriveMode {
    Differential,
    Arcade
  }
}
