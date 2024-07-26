package frc.robot.commands.vision;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.util.List;

public class aimTele extends Command {
    private SwerveSubsystem m_drive;
    private VisionSubsystem m_vision;
    private double[] props;
    private double m_leftXValue;
    private List<Double> m_targets;

    public aimTele(SwerveSubsystem drive, VisionSubsystem vision, double leftXValue, List<Double> targets) {
        m_drive = drive;
        m_vision = vision;
        m_leftXValue = leftXValue;
        m_targets = targets;
        props = new double[2];

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        props = m_vision.limelight_range_and_aim_proportional(m_targets);
        m_drive.aimAtSpeaker(m_leftXValue, props[0], props[1]);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        SmartDashboard.putBoolean("Limelight/ShootNow", false);
        SmartDashboard.putBoolean("Limelight/TargetIDDetected", false);
    }
}