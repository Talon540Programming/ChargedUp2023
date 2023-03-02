package frc.robot.intake.commands;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.constants.Constants;
import frc.robot.intake.IntakeBase;
import frc.robot.intake.IntakeStateManager;

public class IntakeStateController extends CommandBase {
    private final IntakeBase m_intakeBase;

    private final PIDController m_clawPositionController =
            new PIDController(
                    Constants.Intake.ControlValues.ClawPosition.kP,
                    Constants.Intake.ControlValues.ClawPosition.kI,
                    Constants.Intake.ControlValues.ClawPosition.kD);

    private final PIDController m_wristPositionController =
            new PIDController(
                    Constants.Intake.ControlValues.WristPosition.kP,
                    Constants.Intake.ControlValues.WristPosition.kI,
                    Constants.Intake.ControlValues.WristPosition.kD);

    public IntakeStateController(IntakeBase intakeBase) {
        m_intakeBase = intakeBase;
        addRequirements(intakeBase);
    }

    @Override
    public void initialize() {
        m_clawPositionController.reset();
        m_wristPositionController.reset();
    }

    @Override
    public void execute() {
        // Handle Wrist
        m_wristPositionController.setSetpoint(
                IntakeStateManager
                        .getInstance()
                        .getTargetWristState()
                        .angle
        );

        // Handle Claw
        switch (IntakeStateManager.getInstance().getTargetClawState()) {
            case Idle -> {
                m_clawPositionController.setSetpoint(IntakeStateManager.ValidClawState.Idle.angle);

                if(m_intakeBase.m_colorSensorInputs.ProximityValue > 0.6)
                    break;

                IntakeBase.CurrentSeenTarget currentSeenTarget = m_intakeBase.getCurrentSeenTarget();

                if (currentSeenTarget == IntakeBase.CurrentSeenTarget.Unknown)
                    break;

                switch (currentSeenTarget) {
                    case Cone -> IntakeStateManager
                            .getInstance()
                            .updateState(IntakeStateManager.ValidClawState.HoldingCone);

                    case Cube -> IntakeStateManager
                            .getInstance()
                            .updateState(IntakeStateManager.ValidClawState.HoldingCube);

                }
            }
            case HoldingCone -> {
                if(!m_intakeBase.isHoldingCone())
                    DriverStation.reportWarning("Claws are set to hold a CONE, but no CONE is detected in the intake", false);

                if(m_intakeBase.isHoldingCube()) {
                    DriverStation.reportError("Claws are set to hold a CONE, but a CUBE was detected in the intake, stopping to prevent popping it", false);
                    break;
                }

                m_clawPositionController.setSetpoint(IntakeStateManager.ValidClawState.HoldingCone.angle);
            }
            case HoldingCube -> {
                if(!m_intakeBase.isHoldingCube())
                    DriverStation.reportWarning("Claws are set to hold a CUBE, but no CUBE is detected in the intake", false);

                if(m_intakeBase.isHoldingCone())
                    DriverStation.reportWarning("Claws are set to hold a CUBE, but a CONE was detected in the intake", false);

                m_clawPositionController.setSetpoint(IntakeStateManager.ValidClawState.HoldingCube.angle);
            }
        }

        double wristOutput = m_wristPositionController
                .calculate(m_intakeBase.m_wristEncoderInputs.AbsolutePositionRad);
        double clawOutput = m_clawPositionController
                .calculate(m_intakeBase.m_clawEncoderInputs.AbsolutePositionRad);

        m_intakeBase.setWristVoltage(wristOutput);
        m_intakeBase.setClawVoltage(clawOutput);
    }
}
