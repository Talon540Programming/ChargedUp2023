package frc.robot.constants;

import com.pathplanner.lib.PathConstraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public class RobotLimits {
  public static final double kMaxDrivetrainVelocityMetersPerSecond = 4;
  public static final double kMaxDrivetrainAccelerationMetersPerSecondSquared = 3.5;

  public static final PathConstraints kTrajectoryConstraints =
      new PathConstraints(
          kMaxDrivetrainVelocityMetersPerSecond, kMaxDrivetrainAccelerationMetersPerSecondSquared);

  public static final double kMaxArmVelocityRadPerSecond = 3 * Math.PI / 4;
  public static final double kMaxArmAccelerationRadPerSecondSquared = Math.PI / 3;

  public static final TrapezoidProfile.Constraints kArmRotationConstraints =
      new TrapezoidProfile.Constraints(
          kMaxArmVelocityRadPerSecond, kMaxArmAccelerationRadPerSecondSquared);

  public static final double kMinArmLengthInches = RobotDimensions.Arm.kFullyRetractedLengthInches;
  public static final double kMinArmLengthMeters = Units.inchesToMeters(kMinArmLengthInches);

  public static final double kMaxArmLengthInches = RobotDimensions.Arm.kFullyExtendedLengthInches;
  public static final double kMaxArmLengthMeters = Units.inchesToMeters(kMaxArmLengthInches);
}
