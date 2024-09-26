package frc.robot.commands.arm;

import frc.robot.subsystems.ArmSubsystem;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;

public class ArmToPosition extends Command {
    private ArmSubsystem m_arm;
    private ArmSubsystem.armPositions m_targetPos;
    private double armPositionLimelight = .1;
    private boolean m_keepRunning;
    private boolean reachedTarget = false;
    private boolean usingLimeligthArm = false;
    //private double m_startTime = 0;

    public ArmToPosition(ArmSubsystem arm, ArmSubsystem.armPositions pos){
        m_arm = arm;
        m_targetPos = pos;
        m_keepRunning = false;
        armPositionLimelight = .1;
        addRequirements(m_arm);
    }

    public ArmToPosition(ArmSubsystem arm, ArmSubsystem.armPositions pos, boolean keepRunning){
        m_arm = arm;
        m_targetPos = pos;
        m_keepRunning = keepRunning;
        armPositionLimelight = .1;
        addRequirements(m_arm);
    }

    public ArmToPosition(ArmSubsystem arm, double position)
    {
        m_arm = arm;
        armPositionLimelight = position;
        m_keepRunning = false;
        usingLimeligthArm = true;
        addRequirements(m_arm);
    }

    @Override
    public void initialize() {
      //m_startTime = Timer.getFPGATimestamp();
    }
    /* 
    public double getTime() {
        return Timer.getFPGATimestamp() - m_startTime;
    }*/

    @Override
    public void execute(){
        if(usingLimeligthArm == false)
        {
          reachedTarget = m_arm.raiseArmAbs(m_targetPos);
        }
        else
        {   
            reachedTarget = m_arm.raiseArmAbsWithAutoLeveling(armPositionLimelight);
        }
    }

    @Override
    public boolean isFinished(){
        //if(reachedTarget)
        //{
        //    return true;
        //}
        return false;
    }

    @Override
    public void end(boolean isInterrupted){
        //if (!m_keepRunning) m_arm.noArmPower();
    }
}