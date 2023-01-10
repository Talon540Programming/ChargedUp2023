package frc.robot.constants;

import org.talon540.hardware.CANDeviceConfig;

public class HardwareDevices {
  public static final int kDriverXboxControllerId = 0; // TODO
  public static final int kArmControllerXboxControllerId = 0; // TODO

  public static final CANDeviceConfig kRobotGyroConfig = new CANDeviceConfig(0); // TODO

  public static class Drivetrain {
    public static final CANDeviceConfig kFrontLeftConfig = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kFrontRightConfig = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kBackLeftConfig = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kBackRightConfig = new CANDeviceConfig(0); // TODO
  }

  public static class Arm {
    public static final CANDeviceConfig kArmRotationLeader = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kArmRotationFollower = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kArmExtension = new CANDeviceConfig(0); // TODO
    public static final CANDeviceConfig kArmRotationEncoder = new CANDeviceConfig(0); // TODO

    public static int kPhaseOneSensorPort = 0; // TODO
    public static int kPhaseTwoSensorPort = 0; // TODO
    public static int kPhaseThreeSensorPort = 0; // TODO
  }
}
