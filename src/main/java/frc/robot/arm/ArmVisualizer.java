package frc.robot.arm;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.constants.Constants;
import frc.robot.constants.RobotDimensions;
import org.littletonrobotics.junction.Logger;

/**
 * Class used to help log the state of the arm as both Mechanism2d and Mechanism3d objects.
 */
public class ArmVisualizer {
    private final String m_key;

    private final ArmKinematics m_armKinematics;

    private final Mechanism2d m_mechanism;

    private final MechanismLigament2d m_firstExtrusion;
    private final MechanismLigament2d m_secondExtrusion;
    private final MechanismLigament2d m_thirdExtrusion;


    /**
     * Create a ArmVisualizer object.
     * 
     * @param logKey key to log the state of the arm under. Note, this cannot be repeated or incorrect or unintended data may be logged.
     * @param armKinematics the kinematic object for the arm.
     */
    public ArmVisualizer(String logKey, ArmKinematics armKinematics) {
        this.m_key = logKey;
        this.m_armKinematics = armKinematics;

        double fulcrumHeightMeters = armKinematics.getFulcrumPose().getZ();

        m_mechanism = new Mechanism2d(4, 3, new Color8Bit(14, 17, 23));
        MechanismRoot2d fulcrum =
                m_mechanism.getRoot("Arm Fulcrum", 2, fulcrumHeightMeters);

        fulcrum
            .append(new MechanismLigament2d("Arm Upright", fulcrumHeightMeters, -90))
            .setColor(new Color8Bit(Color.kBlue));

        m_firstExtrusion = fulcrum
            .append(
                new MechanismLigament2d(
                        "Arm | First Extrusion",
                        RobotDimensions.Arm.kFirstExtrusionLengthMeters,
                        0,
                        6,
                        new Color8Bit(Color.kRed)));

        m_secondExtrusion = m_firstExtrusion
            .append(
                new MechanismLigament2d(
                        "Arm | Second Extrusion",
                        RobotDimensions.Arm.kSecondExtrusionLengthMeters,
                        0,
                        4,
                        new Color8Bit(Color.kBlack)));
        
        m_thirdExtrusion = m_secondExtrusion
            .append(
                new MechanismLigament2d(
                        "Arm | Third Extrusion",
                        RobotDimensions.Arm.kThirdExtrusionLengthMeters,
                        0,
                        2,
                        new Color8Bit(Color.kWhite)));
        m_thirdExtrusion
            .append(
                new MechanismLigament2d(
                        "Effector",
                        RobotDimensions.Effector.kLengthMeters,
                        0,
                        12,
                        new Color8Bit(Color.kBlueViolet)));
    }

    /**
     * Update the state of the visualizer.
     * 
     * <p>
     * For 2d Objects, the angle of the entire arm is updated and individual ligaments / extrusions / stages are updated to their estimated length based on kinematics.
     * 
     * <p>
     * For 3d Objects, the estimated position of the end of each extrusion / stage is updated.
     * 
     * @param armAngleRad angle of the arm.
     * @param armLengthMeters total distance of the arm from the fulcrum to the end of the effector in meters.
     */
    public void update(double armAngleRad, double armLengthMeters) {
        Translation3d fulcrumTrans = m_armKinematics.getFulcrumPose();

        // Calculate the position of the required components
        Pose3d firstExtrusionPose = m_armKinematics.calculateFirstExtrusionPose(armAngleRad);
        Pose3d secondExtrusionPose = m_armKinematics.calculateSecondExtrusionPose(armLengthMeters, armAngleRad);
        Pose3d thirdExtrusionPose = m_armKinematics.calculateThirdExtrusionPose(armLengthMeters, armAngleRad);
        
        // Update the 2d Ligaments with their correct data
        m_firstExtrusion.setAngle(Math.toDegrees(armAngleRad));

        double firstExtrusionDistance = fulcrumTrans.getDistance(firstExtrusionPose.getTranslation());
        double secondExtrusionDistance = fulcrumTrans.getDistance(secondExtrusionPose.getTranslation());
        double thirdExtrusionDistance = fulcrumTrans.getDistance(thirdExtrusionPose.getTranslation());

        m_secondExtrusion.setLength(secondExtrusionDistance - firstExtrusionDistance);
        m_thirdExtrusion.setLength(thirdExtrusionDistance - secondExtrusionDistance);
    
        // Log the 2d Mechanism
        Logger.getInstance().recordOutput("Arm/Mechanism2d/" + m_key, m_mechanism);
        
        // Log the 3d Mechanism
        Logger.getInstance().recordOutput("Arm/Mechanism3d/" + m_key, firstExtrusionPose, secondExtrusionPose, thirdExtrusionPose);

    }
}
