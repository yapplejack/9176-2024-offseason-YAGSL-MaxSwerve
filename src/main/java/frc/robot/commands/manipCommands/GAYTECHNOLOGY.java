package frc.robot.commands.manipCommands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.BackpackSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class GAYTECHNOLOGY extends Command {
    private FeederSubsystem m_feeder;
    private IndexerSubsystem m_indexer;
    private ShooterSubsystem m_shooter;
    private BackpackSubsystem m_backpack;
    
    public GAYTECHNOLOGY(ShooterSubsystem shooter, FeederSubsystem feeder, IndexerSubsystem indexer, BackpackSubsystem backpack)
    {
        m_feeder = feeder;
        m_indexer = indexer;
        m_shooter = shooter;
        m_backpack = backpack;

        addRequirements(feeder);
        addRequirements(indexer);
        addRequirements(shooter);
        addRequirements(backpack);
    }

     @Override
    public void initialize() {
    }


    @Override
    public void execute() {
        m_shooter.runShooter(-.55);
        m_indexer.RunIndexer(-1);
        m_backpack.runBackpack(-1);

    }

    @Override
    public boolean isFinished() {
        return false;
    }

    @Override
    public void end(boolean isInterrupted) {
        m_feeder.runFeeder(0);
        m_indexer.RunIndexer(0);
        m_backpack.runBackpack(0);
        m_shooter.runShooter(0);
    }
}
