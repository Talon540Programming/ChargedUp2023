package frc.robot.intake.commands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.intake.IntakeBase;

public class IntakeForTime extends CommandBase {
  private final IntakeBase m_intakeBase;

  private final Timer m_timer = new Timer();

  private final double m_time;
  private final double m_voltage;

  public IntakeForTime(IntakeBase intakeBase, double time, double voltage) {
    this.m_intakeBase = intakeBase;

    m_time = Math.max(time, 0);
    m_voltage = MathUtil.clamp(voltage, -12, 12);

    addRequirements(intakeBase);
  }

  @Override
  public void initialize() {
    m_timer.restart();
    m_intakeBase.setVoltage(m_voltage);
  }

  @Override
  public void end(boolean interrupted) {
    m_intakeBase.stopIntake();
    m_timer.stop();
  }

  @Override
  public boolean isFinished() {
    return m_timer.hasElapsed(m_time);
  }
}
