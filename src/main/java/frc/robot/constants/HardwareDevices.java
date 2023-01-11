package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
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

    public static final NeutralMode kArmRotationNeutralMode = NeutralMode.Brake;
    public static final NeutralMode kArmExtensionNeutralMode = NeutralMode.Brake;

    public static int kArmBasePort = 0; // TODO
    public static int kNodeSlotOnePort = 0; // TODO
    public static int kNodeSlotTwoPort = 0; // TODO
    public static int kNodeSlotThreePort = 0; // TODO
    public static int kArmEndPort = 0; // TODO
  }
}
