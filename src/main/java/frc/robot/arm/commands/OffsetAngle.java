package frc.robot.arm.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.arm.ArmBase;
import frc.robot.arm.ArmState;
import frc.robot.constants.RobotLimits;

import java.util.function.DoubleSupplier;

public class OffsetAngle extends CommandBase {
    private final ArmBase m_armBase;
    private final DoubleSupplier m_angleSupplier;

    public OffsetAngle(ArmBase armBase, DoubleSupplier angleSupplier) {
        this.m_armBase = armBase;
        this.m_angleSupplier = angleSupplier;

        addRequirements(armBase);
    }

    @Override
    public void execute() {
        double angle = Math.PI / 2 - m_angleSupplier.getAsDouble();
        m_armBase.updateState(new ArmState(angle, RobotLimits.kMinArmLengthMeters));
    }
}
