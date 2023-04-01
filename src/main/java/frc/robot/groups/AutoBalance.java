package frc.robot.groups;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.arm.ArmBase;
import frc.robot.arm.commands.OffsetAngle;
import frc.robot.drivetrain.DriveBase;
import frc.robot.drivetrain.commands.StabilizeRobot;

public class AutoBalance extends ParallelCommandGroup {
  public AutoBalance(DriveBase driveBase, ArmBase armBase) {
    super(
        new StabilizeRobot(driveBase),
        new OffsetAngle(armBase, () -> driveBase.m_driveInputs.GyroPitchRad));
  }
}
