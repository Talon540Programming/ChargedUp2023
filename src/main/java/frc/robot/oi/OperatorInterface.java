package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Represents the inputs required for the operator or "button man". */
public interface OperatorInterface {
  public double getRotationLinearX();

  public double getRotationLinearY();

  public double getExtensionPercent();

  public Trigger lockRotation();

  public Trigger calibrateExtension();

  public double getIntakePercent();

  public Trigger ejectIntake();

  public Trigger idle();

  public Trigger singleSubstation();

  public Trigger cubeHigh();

  public Trigger cubeMid();

  public Trigger coneMid();

  public Trigger hybrid();
}
