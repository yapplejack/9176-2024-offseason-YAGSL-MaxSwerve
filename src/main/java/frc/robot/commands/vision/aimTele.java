package frc.robot.commands.vision;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;

public class aimTele extends Command {
    private SwerveSubsystem m_drive;

    public aimTele(SwerveSubsystem drive) {
        m_drive = drive;

        addRequirements(m_drive);
    }

    @Override
    public void execute() {
        //m_shooter.spinShootNote(.7,.4);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        //m_shooter.runShooter(0);

    }
}