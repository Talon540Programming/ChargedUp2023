package frc.robot.oi;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

public class PS4Operator implements OperatorInterface {
    private final CommandPS4Controller m_controller;

    private final LoggedDashboardChooser<Double> m_rotationLimiter =
            new LoggedDashboardChooser<>("Arm Rotation Speed Limiter");

    private final LoggedDashboardChooser<Double> m_extensionLimiter =
            new LoggedDashboardChooser<>("Arm Extension Speed Limiter");

    public PS4Operator(int port) {
        this.m_controller = new CommandPS4Controller(port);

        m_rotationLimiter.addDefaultOption("Default (100%)", 1.0);
        m_rotationLimiter.addOption("Fast (70%)", 0.7);
        m_rotationLimiter.addOption("Medium (30%)", 0.3);
        m_rotationLimiter.addOption("Slow (15%)", 0.15);

        m_extensionLimiter.addDefaultOption("Default (100%)", 1.0);
        m_extensionLimiter.addOption("Fast (70%)", 0.7);
        m_extensionLimiter.addOption("Medium (30%)", 0.3);
        m_extensionLimiter.addOption("Slow (15%)", 0.15);
    }

    @Override
    public double getArmRotationPercent() {
        return MathUtil.applyDeadband(m_controller.getLeftY(), 0.05) * m_rotationLimiter.get();
    }

    @Override
    public double getArmExtensionPercent() {
        return MathUtil.applyDeadband(m_controller.getRightY(), 0.05) * m_extensionLimiter.get();
    }
}
