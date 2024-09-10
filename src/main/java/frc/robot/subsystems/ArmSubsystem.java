package frc.robot.subsystems;

import java.util.EnumMap;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkBase.ControlType;
import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder.Type;
import com.revrobotics.SparkPIDController;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;
//import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.NeoMotorConstants;

public class ArmSubsystem extends SubsystemBase {
    public enum armPositions{
        STOWED, 
        AMP, 
        INTAKE,
        SOURCE,
        SUBSHOT,
        PODSHOT,
        POOP,
        PASS,
        AUTOPODSHOT
    }

    private final CANSparkMax m_armRight = new CANSparkMax(NeoMotorConstants.kArmRightDeviceId, MotorType.kBrushless);
    private final CANSparkMax m_armLeft = new CANSparkMax(NeoMotorConstants.kArmLeftDeviceId, MotorType.kBrushless);

    private SparkPIDController m_pidController;

    private final AbsoluteEncoder armAbsEncoder;

    private double currentGoal = 0.125f;

    DoubleSubscriber currentGoalSub;

    private Timer m_timer;

    private double kS_tuner = 0;

    private double kP = 1.2;
    private double kI = .4;
    private double kD = 0.0;

    //.43 kG, 1.17 V*s/rad, .01 V*s^2/rad CALCULATED VALUES

    private double kG = .4;
    private double kS = .4;
    private double kV = 1.95;
    private double kA = .01;

    private boolean isTuning = false;

    private ArmFeedforward ff =
    new ArmFeedforward(kS, kG ,kV, kA);

    private TrapezoidProfile profile;

    private final TrapezoidProfile defaultProfile;

    private final TrapezoidProfile ampProfile;

    private TrapezoidProfile.State setpointState = new TrapezoidProfile.State();

    EnumMap<armPositions, Double> mapAbs = new EnumMap<>(armPositions.class);

    double m_speed = 0.0;

    public ArmSubsystem () {

        m_timer = new Timer();
        m_timer.start();
        m_timer.reset();

        mapAbs.put(armPositions.STOWED, ArmConstants.STOWED);
        mapAbs.put(armPositions.AMP, ArmConstants.AMP);
        mapAbs.put(armPositions.SOURCE, ArmConstants.SOURCE);
        mapAbs.put(armPositions.SUBSHOT, ArmConstants.SUBSHOT);
        mapAbs.put(armPositions.PODSHOT, ArmConstants.PODSHOT);
        mapAbs.put(armPositions.INTAKE, ArmConstants.INTAKE);
        mapAbs.put(armPositions.PASS, ArmConstants.PASS);
        mapAbs.put(armPositions.POOP, ArmConstants.POOP);
        mapAbs.put(armPositions.AUTOPODSHOT, ArmConstants.AUTOPODSHOT);

        m_armRight.setInverted(true);
        m_armRight.setIdleMode(IdleMode.kBrake);
        m_armRight.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
        m_armLeft.setIdleMode(IdleMode.kBrake);
        m_armLeft.setSmartCurrentLimit(ArmConstants.ARM_CURRENT_LIMIT_A);
        m_armLeft.follow(m_armRight, true);

        defaultProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(8, 4));
        
        ampProfile = new TrapezoidProfile(
            new TrapezoidProfile.Constraints(4, 2));

        profile = defaultProfile;
        
        armAbsEncoder = m_armRight.getAbsoluteEncoder(Type.kDutyCycle);
        m_pidController = m_armRight.getPIDController();
        m_pidController.setFeedbackDevice(armAbsEncoder);
        m_pidController.setOutputRange(Constants.ArmConstants.kArmMinOutput, Constants.ArmConstants.kArmMaxOutput);

        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
        m_pidController.setPositionPIDWrappingEnabled(true);
        m_pidController.setPositionPIDWrappingMinInput(0.0f);
        m_pidController.setPositionPIDWrappingMaxInput(1.0f);

        
        m_armRight.burnFlash();
        m_armLeft.burnFlash();

        SmartDashboard.putNumber("Arm/goal", currentGoal);
        SmartDashboard.putNumber("Arm/KP", kP);
        SmartDashboard.putNumber("Arm/KI", kI);
        SmartDashboard.putNumber("Arm/KD", kD);

        SmartDashboard.putNumber("Arm/KS", kS);
        SmartDashboard.putNumber("Arm/KG", kG);
        SmartDashboard.putNumber("Arm/KV", kV);
        SmartDashboard.putNumber("Arm/KA", kA);

        SmartDashboard.putNumber("Arm/KSTuner", kS_tuner);
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("ArmABS Absolute", armAbsEncoder.getPosition());

        if(isTuning)
        {
            tuneNumbers();
        }

        m_speed = m_armRight.getEncoder().getVelocity();
        SmartDashboard.putNumber("setpointState", setpointState.position);
        if(profile.isFinished(m_timer.get() + .05))
        {
            setpointState = new TrapezoidProfile.State(currentGoal, 0);
            updateMotionProfile();
             setpointState =
          profile.calculate(
              m_timer.get(),
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      currentGoal,
                      Constants.ArmConstants.kMinHeightAbs,
                      Constants.ArmConstants.kMaxHeightAbs),
                   0.0));
        }
        else{
        setpointState =
          profile.calculate(
              m_timer.get(),
              setpointState,
              new TrapezoidProfile.State(
                  MathUtil.clamp(
                      currentGoal,
                      Constants.ArmConstants.kMinHeightAbs,
                      Constants.ArmConstants.kMaxHeightAbs),
                   0.0));
        }

        m_armRight.set(kS_tuner);

        m_pidController.setReference(setpointState.position, ControlType.kPosition, 0, ff.calculate(setpointState.position * 2 * Math.PI, setpointState.velocity * 2 * Math.PI));
    }

    private void updateMotionProfile() {
        m_timer.reset();
      }

    public boolean raiseArmAbs(armPositions position){
        if (((armAbsEncoder.getPosition() < ArmConstants.kMinHeightAbs) && (position == armPositions.STOWED)) ||
            ((armAbsEncoder.getPosition() > ArmConstants.kMaxHeightAbs) && (position == armPositions.AMP))) {
            //m_armRight.set(0);
            //return true;
        }

        double ref = mapAbs.get(position);
        if(position == armPositions.AMP && !profile.equals(ampProfile))
        {
            profile = ampProfile;
            m_pidController.setP(1.0);
        }
        else if(position != armPositions.AMP && profile.equals(ampProfile))
        {
            profile = defaultProfile;
            m_pidController.setP(kP);
        }
        currentGoal = ref;
        updateMotionProfile();


        //double pidOut = MathUtil.clamp(
        //    m_AbsPidController.calculate(armAbsEncoder.getPosition(),ref),
        //    Constants.ArmConstants.kArmMinOutput, Constants.ArmConstants.kArmMaxOutput);
        //m_pidController.setReference(setpointState.position, ControlType.kPosition);

        //m_armRight.
            
        //SmartDashboard.putNumber("Arm Abs Target Pos", ref);
 //       m_armRight.set(pidOut);
        
        //if(atPosition(position))
       // {
       //     return true;
       // }
        return false;
    }

    public void updatePID() 
    {
        m_pidController.setP(kP);
        m_pidController.setI(kI);
        m_pidController.setD(kD);
    }

    public boolean atPosition(){
        double currentEncoderPosition = armAbsEncoder.getPosition();
        return (Math.abs(currentEncoderPosition - currentGoal) < Constants.ArmConstants.kAllowedErrAbs);
    }

    public void noArmPower()
    {
        m_armRight.set(0);
    }

    public void tuneNumbers()
    {
        currentGoal = SmartDashboard.getNumber("Arm/goal", currentGoal);
        kP = SmartDashboard.getNumber("Arm/KP", kP);
        kI = SmartDashboard.getNumber("Arm/KI", kI);
        kD = SmartDashboard.getNumber("Arm/KD", kD);

        kS = SmartDashboard.getNumber("Arm/KS", kS); 
        kG = SmartDashboard.getNumber("Arm/KG", kG);
        kV = SmartDashboard.getNumber("Arm/KV", kV);
        kA = SmartDashboard.getNumber("Arm/KA", kA);

        kS_tuner = SmartDashboard.getNumber("Arm/KSTuner", kS_tuner);

        if( m_pidController.getP() != kP || m_pidController.getI() != kI || m_pidController.getD() != kD)
        {
            updatePID();
        }

        if(ff.ka != kA || ff.kg != kG || ff.ks != kS || ff.kv != kV)
        {
            ff = new ArmFeedforward(kS, kG, kV, kA);
        }
    }

}