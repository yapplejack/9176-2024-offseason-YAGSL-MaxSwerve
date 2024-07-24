package frc.robot.commands.swervedrive.drivebase;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class ResetOdometry extends Command {
    private SwerveSubsystem m_drive;

    public ResetOdometry(SwerveSubsystem drive) {
        m_drive = drive;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        
    }

    @Override
    public boolean isFinished() {
        return true;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_drive.resetOdometry(new Pose2d(m_drive.getPose().getX(), m_drive.getPose().getY(), Rotation2d.fromDegrees(180)));
    }
}
