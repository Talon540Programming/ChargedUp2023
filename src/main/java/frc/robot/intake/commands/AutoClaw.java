package frc.robot.intake.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.intake.IntakeBase;

public class AutoClaw extends CommandBase {
  private final IntakeBase m_intakeBase;

  private final PIDController m_positionController =
      new PIDController(
          Constants.Intake.ControlValues.kP,
          Constants.Intake.ControlValues.kI,
          Constants.Intake.ControlValues.kD);

  public AutoClaw(IntakeBase intakeBase) {
    this.m_intakeBase = intakeBase;
    addRequirements(intakeBase);
  }

  @Override
  public void initialize() {
    m_positionController.reset();
  }

  @Override
  public void execute() {
    double setpoint = 0;

    if (m_intakeBase.m_colorSensorInputs.ProximityValue > 0.6) {
      // PID Loop to hold open
      setpoint = Constants.Intake.kIntakeClawMaximumAngleRad;
    } else {
      if (m_intakeBase.isHoldingSomething()) return;

      IntakeBase.CurrentSeenTarget currentSeenTarget = m_intakeBase.getCurrentSeenTarget();

      if (currentSeenTarget == IntakeBase.CurrentSeenTarget.Unknown) return;

      switch (currentSeenTarget) {
        case Cone -> setpoint = Constants.Intake.kConeIntakeAngle;

        case Cube -> setpoint = Constants.Intake.kCubeIntakeAngle;
      }
    }

    setpoint = Math.max(Constants.Intake.kIntakeClawMinimumAngleRad, setpoint);

    double output =
        m_positionController.calculate(
            m_intakeBase.m_clawEncoderInputs.AbsolutePositionRad, setpoint);

    m_intakeBase.setClawVoltage(output);
  }
}
