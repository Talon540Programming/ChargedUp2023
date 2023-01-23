package frc.robot.constants;

import org.talon540.hardware.CANDeviceConfig;

public class HardwareDevices {
  public static final int kDriverXboxControllerPort = 0;

  public static final CANDeviceConfig kRobotGyroConfig = new CANDeviceConfig(0); // TODO

  public static class Drivetrain {
    public static final CANDeviceConfig kLeftLeader = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kLeftFollower = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kRightLeader = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kRightFollower = new CANDeviceConfig(0); // TODO
  }
}
