package frc.robot.arm;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Rotation2d;
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

public class ArmBase extends SubsystemBase {
  // region
  private final ArmExtensionIO m_extensionIO;
  public final ArmExtensionIOInputsAutoLogged m_armExtensionInputs =
      new ArmExtensionIOInputsAutoLogged();

  private final ArmRotationIO m_rotationIO;
  public final ArmRotationIOInputsAutoLogged m_armRotationInputs =
      new ArmRotationIOInputsAutoLogged();

  private ArmState m_targetState = ArmState.IDLE;

  private final ArmVisualizer m_measuredVisualizer =
      new ArmVisualizer("Measured", Constants.Arm.kArmKinematics);
  private final ArmVisualizer m_targetVisualizer =
      new ArmVisualizer("Target", Constants.Arm.kArmKinematics);

  private boolean m_disabled = false;
  private boolean m_disabledVoltageApplied = false;

  private final ProfiledPIDController m_rotationController =
      new ProfiledPIDController(
          Constants.Arm.ControlValues.RotationValues.kP,
          Constants.Arm.ControlValues.RotationValues.kI,
          Constants.Arm.ControlValues.RotationValues.kD,
          RobotLimits.kArmRotationConstraints);

  private final PIDController m_extensionController =
      new PIDController(
          Constants.Arm.ControlValues.ExtensionValues.kP,
          Constants.Arm.ControlValues.ExtensionValues.kI,
          Constants.Arm.ControlValues.ExtensionValues.kD);

  private boolean extensionCalibrated = false;
  // endregion

  public ArmBase(ArmExtensionIO extensionIO, ArmRotationIO rotationIO) {
    this.m_extensionIO = extensionIO;
    this.m_rotationIO = rotationIO;
  }

  public void setDisabled(boolean disabled) {
    this.m_disabled = disabled;
  }

  private boolean armDisabled() {
    return DriverStation.isDisabled() || m_disabled || !extensionCalibrated;
  }

  @Override
  public void periodic() {
    m_extensionIO.updateInputs(m_armExtensionInputs);
    m_rotationIO.updateInputs(m_armRotationInputs);

    Logger.getInstance().processInputs("Arm/Extension", m_armExtensionInputs);
    Logger.getInstance().processInputs("Arm/Rotation", m_armRotationInputs);

    Logger.getInstance().recordOutput("Arm/Extension/Calibrated", extensionCalibrated);
    Logger.getInstance().recordOutput("Arm/Disabled", armDisabled());
    Logger.getInstance().recordOutput("Arm/AtSetpoint", atSetpoint());

    if (armDisabled()) {
      m_targetState = ArmState.IDLE;

      // Only stop movement as soon as arm is disabled instead of continuously while disabled.
      // Ideally, this isn't needed, but because some processes work through direct control, this is
      // needed.
      if (!m_disabledVoltageApplied) {
        stopExtension();
        stopRotation();
        m_disabledVoltageApplied = true;
      }

      // Controllers will be continuously reset while the arm is considered disabled.
      m_rotationController.reset(
          m_armRotationInputs.AbsoluteArmPositionRad, m_armRotationInputs.ArmVelocityRadPerSecond);
      m_extensionController.reset();
    } else {
      m_disabledVoltageApplied = false;

      double rotationFeedforward = ArmSystemDynamics.calculateRotationFeedForward(m_targetState);
      double rotationFeedback =
          m_rotationController.calculate(
              m_armRotationInputs.AbsoluteArmPositionRad, m_targetState.Angle.getRadians());
      m_rotationIO.setVoltage(rotationFeedforward + rotationFeedback);

      double extensionFeedBack =
          m_extensionController.calculate(
              m_armExtensionInputs.PivotToEffectorDistanceMeters,
              m_targetState.PivotToEffectorDistanceMeters);
      m_extensionIO.setVoltage(extensionFeedBack);
    }

    // Update the logged visualizers of the Arm's state
    m_measuredVisualizer.update(
        m_armRotationInputs.AbsoluteArmPositionRad,
        m_armExtensionInputs.PivotToEffectorDistanceMeters);
    m_targetVisualizer.update(
        m_targetState.Angle.getRadians(), m_targetState.PivotToEffectorDistanceMeters);
  }

  @Override
  public void simulationPeriodic() {
    m_rotationIO.updateArmLength(m_armExtensionInputs.PivotToEffectorDistanceMeters);
  }

  public ArmState getCurrentState() {
    return new ArmState(
        Rotation2d.fromRadians(m_armRotationInputs.AbsoluteArmPositionRad),
        m_armRotationInputs.ArmVelocityRadPerSecond,
        m_armExtensionInputs.PivotToEffectorDistanceMeters);
  }

  public ArmState getTargetState() {
    return m_targetState;
  }

  public void updateState(ArmState state) {
    if (state.equals(getTargetState())) return;

    double stateAngleRad = state.Angle.getRadians();
    double stateDistanceMeters = state.PivotToEffectorDistanceMeters;

    double totalLength = stateDistanceMeters + RobotDimensions.Effector.kLengthMeters;

    // Prevent from going through the floor
    if (Math.PI > stateAngleRad && stateAngleRad >= -Math.PI / 2.0) {
      if (Constants.Arm.kArmKinematics.wouldIntersectForward(totalLength, stateAngleRad)) {
        stateAngleRad = Constants.Arm.kArmKinematics.lowestForwardAngle(totalLength);
      }
    } else {
      if (Constants.Arm.kArmKinematics.wouldIntersectRear(totalLength, stateAngleRad)) {
        stateAngleRad = Constants.Arm.kArmKinematics.lowestRearAngle(totalLength);
      }
    }

    // Prevent from breaching extension limit
    if (Constants.Arm.kArmKinematics.wouldBreakExtensionLimit(totalLength, stateAngleRad)) {
      stateDistanceMeters =
          Constants.Arm.kArmKinematics.maxArmAndEffectorLength(stateAngleRad)
              - RobotDimensions.Effector.kLengthMeters;
    }

    // Normalize the angle such that PID won't cause the arm to go through the drivetrain itself
    stateAngleRad %= 2.0 * Math.PI;

    if (stateAngleRad < -Math.PI / 2.0) {
      stateAngleRad += 2 * Math.PI;
    } else if (stateAngleRad > 3.0 * Math.PI / 2.0) {
      stateAngleRad -= 2 * Math.PI;
    }

    stateAngleRad %= 2.0 * Math.PI;

    m_targetState = new ArmState(Rotation2d.fromRadians(stateAngleRad), stateDistanceMeters);
  }

  /**
   * Reset the extension of the arm to all the way down and register the arm's extension encoder to
   * be calibrated. Until extension is calibrated, the arm is unable to work.
   */
  public void completeExtensionCalibration() {
    m_extensionIO.setDistance(RobotDimensions.Arm.kFullyRetractedLengthMeters);
    extensionCalibrated = true;
  }

  public boolean extensionCalibrated() {
    return extensionCalibrated;
  }

  public boolean atSetpoint() {
    return getCurrentState().equals(getTargetState());
  }

  public void stopExtension() {
    m_extensionIO.setVoltage(0.0);
  }

  public void stopRotation() {
    m_rotationIO.setVoltage(0.0);
  }

  public void setRotationVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    m_rotationIO.setVoltage(voltage);
  }

  public void setExtensionVoltage(double voltage) {
    voltage = MathUtil.clamp(voltage, -12, 12);
    m_extensionIO.setVoltage(voltage);
  }

  public boolean isExtensionStalled() {
    return m_extensionIO.isStalled();
  }
}
