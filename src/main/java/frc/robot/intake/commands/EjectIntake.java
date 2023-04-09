package frc.robot.intake.commands;

import frc.robot.intake.IntakeBase;

public class EjectIntake extends IntakeForTime {
  public EjectIntake(IntakeBase intakeBase) {
    super(intakeBase, 0.75, -4);
  }
}
