package frc.robot.oi;

import edu.wpi.first.wpilibj2.command.button.Trigger;

/** Represents the inputs required for the operator or "button man". */
public interface OperatorInterface {
  public double getRotationLinearX();

  public double getRotationLinearY();

  public double getExtensionPercent();

  public Trigger lockRotation();

  public Trigger resetExtension();

  public double getIntakePercent();
}
