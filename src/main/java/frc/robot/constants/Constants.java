package frc.robot.constants;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public class Constants {
  public static class Arm {
    public static class Extension {
      public static final double kArmExtensionRate = 0.25; // TODO
    }

    public static class Rotation {
      public static final double kMaxVelocityRadiansPerSecond = 0; // TODO
      public static final double kMaxAccelerationRadiansPerSecondSquared = 0; // TODO

      public static final TrapezoidProfile.Constraints kRotationConstraints =
          new TrapezoidProfile.Constraints(
              kMaxVelocityRadiansPerSecond, kMaxAccelerationRadiansPerSecondSquared);

      // Gearbox * Sprocket
      public static final double kGearRatio = 0; // TODO

      public static class ControlValues {
        public static double kP = 0; // TODO
        public static double kI = 0; // TODO
        public static double kD = 0; // TODO
        public static double kS = 0; // TODO
        public static double kG = 0; // TODO
        public static double kV = 0; // TODO
        public static double kA = 0; // TODO
      }
    }
  }
}
