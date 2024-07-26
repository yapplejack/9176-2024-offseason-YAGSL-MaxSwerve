package frc.robot.commands.shooter;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;

public class PassShooter extends Command {
    private ShooterSubsystem m_shooter;

    public PassShooter(ShooterSubsystem shooter) {
        m_shooter = shooter;

        addRequirements(m_shooter);
    }

    @Override
    public void execute() {
        m_shooter.spinShootNote(.5,.35); //.9, .8 
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_shooter.runShooter(0);

    }
}