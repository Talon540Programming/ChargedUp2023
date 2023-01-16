package frc.robot.constants;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.sensors.AbsoluteSensorRange;
import com.ctre.phoenix.sensors.CANCoderConfiguration;
import com.ctre.phoenix.sensors.SensorTimeBase;
import org.talon540.hardware.CANDeviceConfig;

public class HardwareDevices {
  public static class Arm {
    public static class Extension {
      public static final CANDeviceConfig kArmExtension = new CANDeviceConfig(0); // TODO

      public static final NeutralMode kArmExtensionNeutralMode = NeutralMode.Brake;

      public static int kArmBasePort = 0; // TODO
      public static int kNodeSlotOnePort = 0; // TODO
      public static int kNodeSlotTwoPort = 0; // TODO
      public static int kNodeSlotThreePort = 0; // TODO
      public static int kArmEndPort = 0; // TODO
    }

    public static class Rotation {
      public static final CANDeviceConfig kArmRotationLeader = new CANDeviceConfig(0); // TODO
      public static final CANDeviceConfig kArmRotationFollower = new CANDeviceConfig(0); // TODO
      public static final CANDeviceConfig kArmRotationEncoder = new CANDeviceConfig(0); // TODO

      public static CANCoderConfiguration getRotationEncoderConfig() {
        CANCoderConfiguration config = new CANCoderConfiguration();

        config.sensorCoefficient = 2 * Math.PI / 4096.0;
        config.unitString = "rad";
        config.sensorTimeBase = SensorTimeBase.PerSecond;
        config.absoluteSensorRange = AbsoluteSensorRange.Signed_PlusMinus180;

        return config;
      }

      public static final NeutralMode kArmRotationNeutralMode = NeutralMode.Brake;

      public static int kForwardBeamBreakPort = 0; // TODO
      public static int kRearBeamBreakPort = 0; // TODO
    }
  }
}
