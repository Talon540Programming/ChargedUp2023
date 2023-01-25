package frc.lib.logging;

/**
 * Represents an IO interface that can control a {@link
 * org.littletonrobotics.junction.inputs.LoggableInputs}.
 *
 * @param <T> Loggable Inputs that are updated.
 */
public interface LoggedIO<T> {

  /**
   * Update the inputs with the current state of the IO.
   *
   * @param inputs inputs to update.
   */
  public void updateInputs(T inputs);
}
