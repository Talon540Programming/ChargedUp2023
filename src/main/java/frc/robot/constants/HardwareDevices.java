package frc.robot.constants;

import org.talon540.hardware.CANDeviceConfig;

public final class HardwareDevices {
  public static final int kDriverXboxControllerPort = 0;
  public static final int kDepositionXboxControllerPort = 1;

  public static final class PROTO2023 {
    public static final CANDeviceConfig kRobotGyroConfig = new CANDeviceConfig(1);

    public static class Drivetrain {
      public static final CANDeviceConfig kLeftLeader = new CANDeviceConfig(5);
      public static final CANDeviceConfig kLeftFollower = new CANDeviceConfig(6);
      public static final CANDeviceConfig kRightLeader = new CANDeviceConfig(4);
      public static final CANDeviceConfig kRightFollower = new CANDeviceConfig(3);
    }
  }

  public static final class COMP2023 {
    public static final CANDeviceConfig kRobotGyroConfig = new CANDeviceConfig(1);

    public static class Drivetrain {
      public static final CANDeviceConfig kLeftLeader = new CANDeviceConfig(0); // TODO
      public static final CANDeviceConfig kLeftFollower = new CANDeviceConfig(0); // TODO
      public static final CANDeviceConfig kRightLeader = new CANDeviceConfig(0); // TODO
      public static final CANDeviceConfig kRightFollower = new CANDeviceConfig(0); // TODO
    }
  }

  public static final class BOT2020 {
    public static final CANDeviceConfig kRobotGyroConfig = new CANDeviceConfig(1);

    public static class Drivetrain {
      public static final CANDeviceConfig kLeftLeader = new CANDeviceConfig(8);
      public static final CANDeviceConfig kLeftFollower = new CANDeviceConfig(9);
      public static final CANDeviceConfig kRightLeader = new CANDeviceConfig(10);
      public static final CANDeviceConfig kRightFollower = new CANDeviceConfig(11);
    }
  }
}
