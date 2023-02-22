package frc.robot.arm.commands;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmStateManager;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotLimits;

public class StateController extends CommandBase {
    private final ArmBase m_armBase;

    private final ProfiledPIDController m_rotationController;
    private final ArmFeedforward m_rotationFeedforward;

    private final PIDController m_extensionController;

    public StateController(ArmBase armBase) {
        this.m_armBase = armBase;

        this.m_rotationController =
                new ProfiledPIDController(
                        Constants.Arm.ControlValues.RotationValues.kP,
                        Constants.Arm.ControlValues.RotationValues.kI,
                        Constants.Arm.ControlValues.RotationValues.kD,
                        new TrapezoidProfile.Constraints(
                                RobotLimits.kMaxArmVelocityRadPerSecond,
                                RobotLimits.kMaxArmAccelerationRadPerSecondSquared));
        this.m_rotationFeedforward =
                new ArmFeedforward(
                        Constants.Arm.ControlValues.RotationValues.kS,
                        Constants.Arm.ControlValues.RotationValues.kG,
                        Constants.Arm.ControlValues.RotationValues.kV,
                        Constants.Arm.ControlValues.RotationValues.kA);

        this.m_extensionController =
                new PIDController(
                        Constants.Arm.ControlValues.ExtensionValues.kP,
                        Constants.Arm.ControlValues.ExtensionValues.kI,
                        Constants.Arm.ControlValues.ExtensionValues.kD);

        addRequirements(armBase);
    }

    @Override
    public void initialize() {
        m_rotationController.reset(m_armBase.m_rotationEncoderInputs.AbsolutePositionRad);
        m_extensionController.reset();
    }

    @Override
    public void execute() {
        // Get the target Arm State
        ArmState targetState = ArmStateManager.getInstance().getArmState();

        setExtensionOutput(m_extensionController.calculate(m_armBase.m_armExtensionInputs.DistanceTraveledMeters, targetState.ExtensionLengthMeters));

        TrapezoidProfile.State rotationGoal = targetState.getRotationState();
        setRotationOutput(m_rotationController.calculate(m_armBase.m_rotationEncoderInputs.AbsolutePositionRad, rotationGoal), m_rotationController.getSetpoint());
    }

    /**
     * Set arm rotation voltage based on the output of a PID controller and a setpoint in the form of
     * a {@link TrapezoidProfile.State}. Calculates feedforward values and sets the output of the
     * motors.
     *
     * @param output PID based output in volts.
     * @param setpoint setpoint of the arm's position. Used for FeedForward calculations.
     */
    private void setRotationOutput(double output, TrapezoidProfile.State setpoint) {
        double feedForwardValue = m_rotationFeedforward.calculate(setpoint.position, setpoint.velocity);
        m_armBase.setRotationVoltage(feedForwardValue + output);
    }

    /**
     * Set arm rotation voltage based on the output of a PID controller.
     *
     * @param output PID based output in volts.
     */
    private void setExtensionOutput(double output) {
        m_armBase.setExtensionVoltage(output);
    }
}
