package frc.robot.arm;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIOInputsAutoLogged;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIOInputsAutoLogged;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotLimits;
import frc.robot.sensors.encoder.QuadratureEncoderIO;
import frc.robot.sensors.encoder.QuadratureEncoderIOInputsAutoLogged;
import org.littletonrobotics.junction.Logger;

public class ArmBase extends SubsystemBase {
  private final ArmExtensionIO m_armExtensionIO;
  public final ArmExtensionIOInputsAutoLogged m_armExtensionInputs =
      new ArmExtensionIOInputsAutoLogged();

  private final ArmRotationIO m_armRotationIO;
  public final ArmRotationIOInputsAutoLogged m_armRotationInputs =
      new ArmRotationIOInputsAutoLogged();

  private final QuadratureEncoderIO m_rotationEncoderIO;
  public final QuadratureEncoderIOInputsAutoLogged m_rotationEncoderInputs =
      new QuadratureEncoderIOInputsAutoLogged();

  private final ProfiledPIDController m_rotationController;
  private final ArmFeedforward m_rotationFeedforward;

  private final PIDController m_extensionController;

  public ArmBase(
      ArmExtensionIO extensionIO, ArmRotationIO rotationIO, QuadratureEncoderIO rotationEncoderIO) {
    this.m_armExtensionIO = extensionIO;
    this.m_armRotationIO = rotationIO;
    this.m_rotationEncoderIO = rotationEncoderIO;

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
  }

  @Override
  public void periodic() {
    m_armExtensionIO.updateInputs(m_armExtensionInputs);
    Logger.getInstance().processInputs("Arm/Extension", m_armExtensionInputs);

    m_armRotationIO.updateInputs(m_armRotationInputs);
    Logger.getInstance().processInputs("Arm/Rotation", m_armRotationInputs);

    m_rotationEncoderIO.updateInputs(m_rotationEncoderInputs);
    Logger.getInstance().processInputs("Arm/Rotation/Encoder", m_rotationEncoderInputs);

    // Get the target Arm State
    ArmState targetState = ArmStateManager.getInstance().getArmState();

    // Log the target state
    Logger.getInstance().processInputs("Arm/TargetState", targetState);

    TrapezoidProfile.State rotationGoal = targetState.getRotationState();
    setExtensionOutput(
        m_extensionController.calculate(
            m_armExtensionInputs.DistanceTraveledMeters, targetState.ExtensionLengthMeters));
    setRotationOutput(
        m_rotationController.calculate(m_rotationEncoderInputs.AbsolutePositionRad, rotationGoal),
        rotationGoal);
  }

  /**
   * Set the voltage output of the rotation motors.
   *
   * @param voltage voltage to set.
   */
  public void setRotationVoltage(double voltage) {
    m_armRotationIO.setVoltage(voltage);
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
    setRotationVoltage(feedForwardValue + output);
  }

  /**
   * Set the voltage output of the extension motors.
   *
   * @param voltage voltage to set.
   */
  public void setExtensionVoltage(double voltage) {
    m_armExtensionIO.setVoltage(voltage);
  }

  /**
   * Set arm rotation voltage based on the output of a PID controller.
   *
   * @param output PID based output in volts.
   */
  private void setExtensionOutput(double output) {
    setExtensionVoltage(output);
  }
}
