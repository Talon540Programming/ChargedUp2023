package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Represents the inputs required for the primary driver. */
public interface DriverInterface {
  public DriveMode getDriveMode();

  public double getLeftPercent();

  public double getRightPercent();

  public Trigger toggleBalanceMode();

  public enum DriveMode {
    Differential,
    Arcade
  }
}
