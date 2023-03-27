package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.drivetrain.DriveBase;

public class ScoreCubeHybridTaxi extends SequentialCommandGroup {
  public ScoreCubeHybridTaxi(DriveBase driveBase) {
    super(new DriveTime(driveBase, 0.75, -0.5), new DriveTime(driveBase, 4, 0.25));
  }
}
