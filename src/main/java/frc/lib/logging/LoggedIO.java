// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.lib.logging;

/**
 * Represents an IO interface that can control a {@link
 * org.littletonrobotics.junction.inputs.LoggableInputs}.
 *
 * @param <T> Loggable inputs that are updated.
 */
public interface LoggedIO<T> {

  /**
   * Update the inputs with the current state of the IO.
   *
   * @param inputs inputs to update.
   */
  void updateInputs(T inputs);
}
