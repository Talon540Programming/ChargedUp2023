package frc.robot.constants;

import edu.wpi.first.wpilibj.I2C;

public final class HardwareDevices {
  public static final int kDriverXboxControllerPort = 0;
  public static final int kDepositionXboxControllerPort = 1;

  public static final class PROTO2023 {
    public static final int kRobotGyroId = 11; // TODO ReID gyro

    public static class Drivetrain {
      public static final int kLeftLeaderId = 5;
      public static final int kLeftFollowerId = 6;
      public static final int kRightLeaderId = 4;
      public static final int kRightFollowerId = 3;
    }

    public static class Arm {
      // Proto doesn't have an Arm
      public static final int kRotationLeaderId = -1;
      public static final int kRotationFollowerId = -1;
      public static final int kExtensionId = -1;

      public static final int kArmRotationEncoderId = -1;
    }
  }

  public static final class COMP2023 {
    public static final int kRobotGyroId = 11; // TODO ReID gyro

    public static class Drivetrain {
      public static final int kLeftLeaderId = 5;
      public static final int kLeftFollowerId = 6;
      public static final int kRightLeaderId = 4;
      public static final int kRightFollowerId = 3;
    }

    public static class Arm {
      public static final int kRotationLeaderId = 13;
      public static final int kRotationFollowerId = 12;
      public static final int kExtensionId = 0; // TODO

      public static final int kArmRotationEncoderId = 0; // TODO
    }

    public static class Intake {
      public static final int kIntakeWristId = 0; // TODO
      public static final int kIntakeClawId = 0; // TODO

      public static final int kIntakeWristEncoderId = 0; // TODO
      public static final int kIntakeClawEncoderID = 0; // TODO

      public static final I2C.Port kColorSensorPort = I2C.Port.kMXP;
    }
  }
}
