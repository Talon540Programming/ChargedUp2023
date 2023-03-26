package frc.robot.drivetrain.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.drivetrain.DriveBase;
import java.util.function.BooleanSupplier;

public class DriveTillCondition extends CommandBase {
  private final double kSpeed;

  private final DriveBase m_driveBase;
  private final BooleanSupplier m_supplier;

  /**
   * Create a DriveTime Auto.
   *
   * @param driveBase drivetrain subsystem
   * @param condition supplier condition to check
   * @param speed speed to drive at [-1.0, 1.0]
   */
  public DriveTillCondition(DriveBase driveBase, BooleanSupplier condition, double speed) {
    kSpeed = MathUtil.clamp(speed, -1, 1);
    m_supplier = condition;
    m_driveBase = driveBase;

    addRequirements(driveBase);
  }

  @Override
  public void initialize() {
    m_driveBase.tankDrivePercent(kSpeed, kSpeed);
  }

  @Override
  public void end(boolean interrupted) {
    m_driveBase.stop();
  }

  @Override
  public boolean isFinished() {
    return m_supplier.getAsBoolean();
  }
}
