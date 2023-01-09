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

}
