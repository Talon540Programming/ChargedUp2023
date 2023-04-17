package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Represents the inputs required for the operator or "button man". */
public interface OperatorInterface {
  double getRotationLinearX();

  double getRotationLinearY();

  double getExtensionPercent();

  Trigger lockRotation();

  Trigger calibrateExtension();

  double getIntakePercent();

  Trigger ejectIntake();

  Trigger idle();

  Trigger singleSubstation();

  Trigger cubeHigh();

  Trigger cubeMid();

  Trigger coneMid();

  Trigger hybrid();
}
