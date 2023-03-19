package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.arm.extension.ArmExtensionIO;
import frc.robot.arm.extension.ArmExtensionIOInputsAutoLogged;
import frc.robot.arm.rotation.ArmRotationIO;
import frc.robot.arm.rotation.ArmRotationIOInputsAutoLogged;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotDimensions;
import frc.robot.constants.RobotLimits;
import org.littletonrobotics.junction.Logger;
import java.util.function.BooleanSupplier;

public class ArmBase extends SubsystemBase {
  // region
  private final ArmExtensionIO m_armExtensionIO;
  public final ArmExtensionIOInputsAutoLogged m_armExtensionInputs = new ArmExtensionIOInputsAutoLogged();

  private final ArmRotationIO m_armRotationIO;
  public final ArmRotationIOInputsAutoLogged m_armRotationInputs = new ArmRotationIOInputsAutoLogged();

  private ArmState m_targetState = ArmState.IDLE;

  private final ArmVisualizer m_measuredVisualizer = new ArmVisualizer("Measured", Constants.Arm.kArmKinematics);
  private final ArmVisualizer m_targetVisualizer = new ArmVisualizer("Target", Constants.Arm.kArmKinematics);

  private boolean m_disabled = false;

  private final ProfiledPIDController m_rotationController =
        new ProfiledPIDController(
          Constants.Arm.ControlValues.RotationValues.kP,
          Constants.Arm.ControlValues.RotationValues.kI,
          Constants.Arm.ControlValues.RotationValues.kD,
            new TrapezoidProfile.Constraints(
                  RobotLimits.kMaxArmVelocityRadPerSecond,
                  RobotLimits.kMaxArmAccelerationRadPerSecondSquared
            ));
  private final ArmFeedforward m_rotationFeedforward =
          new ArmFeedforward(
                  Constants.Arm.ControlValues.RotationValues.kS,
                  Constants.Arm.ControlValues.RotationValues.kG,
                  Constants.Arm.ControlValues.RotationValues.kV,
                  Constants.Arm.ControlValues.RotationValues.kA);

  private final PIDController m_extensionController =
        new PIDController(
          Constants.Arm.ControlValues.ExtensionValues.kP,
          Constants.Arm.ControlValues.ExtensionValues.kI,
          Constants.Arm.ControlValues.ExtensionValues.kD);
  // endregion

  public ArmBase(ArmExtensionIO extensionIO, ArmRotationIO rotationIO) {
    this.m_armExtensionIO = extensionIO;
    this.m_armRotationIO = rotationIO;
  }

  public void setDisabled(boolean disabled) {
    this.m_disabled = disabled;
  }

  private boolean armDisabled() {
    return DriverStation.isDisabled() || m_disabled;
  }

  @Override
  public void periodic() {
    m_armExtensionIO.updateInputs(m_armExtensionInputs);
    m_armRotationIO.updateInputs(m_armRotationInputs);

    Logger.getInstance().processInputs("Arm/Extension", m_armExtensionInputs);
    Logger.getInstance().processInputs("Arm/Rotation", m_armRotationInputs);

    // Log the target state
    Logger.getInstance().processInputs("Arm/TargetState", m_targetState);

    m_measuredVisualizer.update(m_armRotationInputs.AbsoluteArmPositionRad, m_armExtensionInputs.PivotToEffectorDistanceMeters);
    m_targetVisualizer.update(m_targetState.AngleRadians, m_targetState.LengthMeters);

    if(armDisabled()) {
      m_targetState = ArmState.IDLE;
      m_armExtensionIO.setVoltage(0.0);
      m_armRotationIO.setVoltage(0.0);

      // Controllers will be continuously reset while the arm is considered disabled.
      m_rotationController.reset(m_armRotationInputs.AbsoluteArmPositionRad);
      m_extensionController.reset();
    } else {
      double rotationOutputVolts = m_rotationController.calculate(m_armRotationInputs.AbsoluteArmPositionRad, m_targetState.AngleRadians);
      rotationOutputVolts += m_rotationFeedforward.calculate(m_armRotationInputs.AbsoluteArmPositionRad, 0);
      m_armRotationIO.setVoltage(rotationOutputVolts);

      double extensionOutputVolts = m_extensionController.calculate(m_armExtensionInputs.PivotToEffectorDistanceMeters, m_targetState.LengthMeters);
      m_armExtensionIO.setVoltage(extensionOutputVolts);
    }
  }

  @Override
  public void simulationPeriodic() {
    m_armRotationIO.updateArmLength(m_armExtensionInputs.PivotToEffectorDistanceMeters);
  }

  public ArmState getTargetState() {
    try {
      return m_targetState.clone();
    } catch (CloneNotSupportedException e) {
      throw new RuntimeException(e);
    }
  }

  public void updateState(ArmState state) {
    state.LengthMeters = MathUtil.clamp(state.LengthMeters, RobotLimits.kMinArmLengthMeters, RobotLimits.kMaxArmLengthMeters);

    double totalLength = state.LengthMeters + RobotDimensions.Effector.kLengthMeters;

    // Prevent from going through the floor
    if(Math.PI > state.AngleRadians && state.AngleRadians >= -Math.PI / 2.0) {
      if(Constants.Arm.kArmKinematics.wouldIntersectForward(totalLength, state.AngleRadians)) {
        state.AngleRadians = Constants.Arm.kArmKinematics.lowestForwardAngle(totalLength);
      }
    } else {
      if(Constants.Arm.kArmKinematics.wouldIntersectRear(totalLength, state.AngleRadians)) {
        state.AngleRadians = Constants.Arm.kArmKinematics.lowestRearAngle(totalLength);
      }
    }

    // Prevent from breaching extension limit
    if(Constants.Arm.kArmKinematics.wouldBreakExtensionLimit(totalLength, state.AngleRadians)) {
        state.LengthMeters = Constants.Arm.kArmKinematics.maxArmAndEffectorLength(state.AngleRadians) - RobotDimensions.Effector.kLengthMeters;
    }

    m_targetState = new ArmState(state.AngleRadians, state.LengthMeters);
  }

  /**
   * Get the length of the arm including the effector.
   *
   * @return total length of the arm system.
   */
  public double getTotalArmLength() {
    return m_armExtensionInputs.PivotToEffectorDistanceMeters
        + RobotDimensions.Effector.kLengthMeters;
  }
}
