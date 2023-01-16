package frc.robot.arm.rotation;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.constants.Constants;
import frc.robot.constants.Constants.Arm.Rotation.ControlValues;
import frc.robot.constants.HardwareDevices;

public class ProfiledArmRotationBase extends ProfiledPIDSubsystem {
    private final WPI_TalonFX rotationLeader = new WPI_TalonFX(HardwareDevices.Arm.Rotation.kArmRotationLeader.id, HardwareDevices.Arm.Rotation.kArmRotationLeader.controller);
    private final WPI_TalonFX rotationFollower = new WPI_TalonFX(HardwareDevices.Arm.Rotation.kArmRotationFollower.id, HardwareDevices.Arm.Rotation.kArmRotationFollower.controller);

    private final WPI_CANCoder rotationEncoder = new WPI_CANCoder(HardwareDevices.Arm.Rotation.kArmRotationEncoder.id, HardwareDevices.Arm.Rotation.kArmRotationEncoder.controller);

    private final DigitalInput forwardBeamBreak = new DigitalInput(HardwareDevices.Arm.Rotation.kForwardBeamBreakPort);
    private final DigitalInput rearBeamBreak = new DigitalInput(HardwareDevices.Arm.Rotation.kRearBeamBreakPort);

    private final ArmFeedforward m_feedForward = new ArmFeedforward(
            ControlValues.kS, ControlValues.kG, ControlValues.kV, ControlValues.kA);


    public ProfiledArmRotationBase() {
        super( new ProfiledPIDController(
                ControlValues.kP,
                ControlValues.kI,
                ControlValues.kD,
                Constants.Arm.Rotation.kRotationConstraints
        ), 0);

        // Configure rotation motors
        this.rotationLeader.setNeutralMode(HardwareDevices.Arm.Rotation.kArmRotationNeutralMode);
        this.rotationFollower.setNeutralMode(HardwareDevices.Arm.Rotation.kArmRotationNeutralMode);

        this.rotationFollower.follow(rotationLeader);

        this.rotationEncoder.configFactoryDefault();
        this.rotationEncoder.configAllSettings(HardwareDevices.Arm.Rotation.getRotationEncoderConfig());
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        double feedForwardValue = m_feedForward.calculate(setpoint.position, setpoint.velocity);
        rotationLeader.setVoltage(output + feedForwardValue);
    }

    @Override
    protected double getMeasurement() {
        return getArmPosition();
    }

    /**
     * Set the output percent of the arm rotation motor. Automatically sets the second motor with
     * follower mode.
     *
     * @param percent percent to set [-1, 1].
     */
    public void setRotationPercent(double percent) {
        rotationLeader.set(ControlMode.PercentOutput, percent);
    }

    /**
     * Return the position of the arm from the rotation encoder.
     *
     * @return position of the arm in radians.
     */
    public double getArmPosition() {
        return rotationEncoder.getPosition();
    }

    /**
     * Return the velocity of arm rotation.
     *
     * @return velocity of arm rotation in radians per second.
     */
    public double getArmVelocity() {
        return rotationEncoder.getVelocity();
    }

    /**
     * Return if the forward beam break sensor is broken. Useful for stopping the arm from forcing
     * itself past this point and damaging itself.
     *
     * @return if the forward beam break sensor is true.
     */
    public boolean isAtForwardLimit() {
        return !forwardBeamBreak.get();
    }

    /**
     * Return if the rear beam break sensor is broken. Useful for stopping the arm from forcing itself
     * past this point and damaging itself.
     *
     * @return if the rear beam break sensor is true.
     */
    public boolean isAtRearLimit() {
        return !rearBeamBreak.get();
    }
}
