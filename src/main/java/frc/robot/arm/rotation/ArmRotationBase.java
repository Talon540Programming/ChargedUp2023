package frc.robot.arm.rotation;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.constants.HardwareDevices.Arm;

public class ArmRotationBase extends SubsystemBase {
    private final WPI_TalonFX rotationLeader;
    private final WPI_CANCoder rotationEncoder = new WPI_CANCoder(Arm.kArmRotationEncoder.id, Arm.kArmRotationEncoder.controller);

    public ArmRotationBase() {
        this.rotationLeader = new WPI_TalonFX(Arm.kArmRotationLeader.id, Arm.kArmRotationLeader.controller);
        WPI_TalonFX rotationFollower = new WPI_TalonFX(Arm.kArmRotationFollower.id, Arm.kArmRotationFollower.controller);
        rotationFollower.follow(rotationLeader);
    }
}
