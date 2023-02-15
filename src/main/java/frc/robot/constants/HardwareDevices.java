package frc.robot.constants;

public final class HardwareDevices {
  public static final int kDriverXboxControllerPort = 0;
  public static final int kDepositionXboxControllerPort = 1;

  public static final class PROTO2023 {
    public static final int kRobotGyroConfig = 1;

    public static class Drivetrain {
      public static final int kLeftLeader = 5;
      public static final int kLeftFollower = 6;
      public static final int kRightLeader = 4;
      public static final int kRightFollower = 3;
    }
  }

  public static final class COMP2023 {
    public static final int kRobotGyroConfig = 0; // TODO

    public static class Drivetrain {
      public static final int kLeftLeader = 0; // TODO
      public static final int kLeftFollower = 0; // TODO
      public static final int kRightLeader = 0; // TODO
      public static final int kRightFollower = 0; // TODO
    }
  }
}
