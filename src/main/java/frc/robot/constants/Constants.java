package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
  public static class Arm {
    public static class Extension {
      public static final double kArmExtensionRate = 0.25; // TODO
    }

    public static class Rotation {
      public static final double kMaxVelocityRadPerSec = 0; // TODO
      public static final double kMaxAccelerationRadPerSecSquared = 0; // TODO

      public static final TrapezoidProfile.Constraints kRotationalConstraint = new TrapezoidProfile.Constraints(
              kMaxVelocityRadPerSec,
              kMaxAccelerationRadPerSecSquared
      );

      // Gearbox * Sprocket
      public static final double kGearRatio = 0; // TODO

      public static class ControlValues {
        public static double kP = 0; // TODO
        public static double kI = 0; // TODO
        public static double kD = 0; // TODO
        public static double kS = 0; // TODO
        public static double kV = 0; // TODO
      }
    }
  }
}
