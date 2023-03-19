package frc.robot.oi;

/** Represents the inputs required for the operator or "button man". */
public interface OperatorInterface {
  public double getArmRotationPercent();

  public double getArmExtensionPercent();
}
