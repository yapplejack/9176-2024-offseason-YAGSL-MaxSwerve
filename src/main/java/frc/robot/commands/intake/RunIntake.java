package frc.robot.commands.intake;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.IntakeSubsystem;

public class RunIntake extends Command {
    private IntakeSubsystem m_intake;

    public RunIntake(IntakeSubsystem intake) {
        m_intake = intake;

        addRequirements(m_intake);
    }

    @Override
    public void execute() {
        m_intake.runIntake(1);
    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_intake.runIntake(0);

    }
}