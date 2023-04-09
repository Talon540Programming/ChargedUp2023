package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import java.util.function.Supplier;

public class GoToSuppliedState extends CommandBase {
  private final ArmBase m_armBase;
  private final Supplier<ArmState> m_stateSupplier;

  public GoToSuppliedState(ArmBase armBase, Supplier<ArmState> stateSupplier) {
    m_armBase = armBase;
    m_stateSupplier = stateSupplier;

    addRequirements(armBase);
  }

  @Override
  public void execute() {
    m_armBase.updateState(m_stateSupplier.get());
  }
}
