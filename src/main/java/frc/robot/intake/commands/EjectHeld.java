package frc.robot.intake.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeBase;

public class EjectHeld extends CommandBase {
    private final IntakeBase m_intakeBase;

    private final Timer m_timer = new Timer();

    public EjectHeld(IntakeBase intakeBase) {
        this.m_intakeBase = intakeBase;
        addRequirements(intakeBase);
    }

    @Override
    public void initialize() {
        m_timer.restart();
        m_intakeBase.setVoltage(8.0);
    }

    @Override
    public void end(boolean interrupted) {
        m_intakeBase.stopIntake();
        m_timer.stop();
    }

    @Override
    public boolean isFinished() {
        return m_timer.hasElapsed(1.25);
    }
}
