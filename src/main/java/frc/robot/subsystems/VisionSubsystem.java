package frc.robot.subsystems;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.LimelightHelpers;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import java.util.List;

public class VisionSubsystem extends SubsystemBase {
    NetworkTable table;
    NetworkTableEntry tx;
    NetworkTableEntry ty;
    NetworkTableEntry ta;
    boolean rotateToAlignWithAmp = false; 
    public VisionSubsystem()
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
    }

    @Override
    public void periodic() {
        //read values periodically
        double x = tx.getDouble(0.0);
        double y = ty.getDouble(0.0);
        double area = ta.getDouble(0.0);

        //post to smart dashboard periodically
        SmartDashboard.putNumber("LimelightX", x);
        SmartDashboard.putNumber("LimelightY", y);
        SmartDashboard.putNumber("LimelightArea", area);
    }
    public double limelight_aim_proportional()
  {    
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kP = .08; // .025
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    double targetingAngularVelocity = LimelightHelpers.getTX("limelight") * kP;

    SmartDashboard.putNumber("tartegtingAngularVelo", targetingAngularVelocity);

    // convert to radians per second for our drive method
    targetingAngularVelocity *=  6.283185307179586;

    //invert since tx is positive when the target is to the right of the crosshair
    targetingAngularVelocity *= -1.0;

    return targetingAngularVelocity;
  }
  public double limelight_range_proportional()
  {    
    double kP = .1; // .08
    double targetingForwardSpeed = LimelightHelpers.getTY("limelight") * kP;
    targetingForwardSpeed *= Constants.MAX_SPEED;
    targetingForwardSpeed *= -1.0;
    return targetingForwardSpeed;
  }

  /**
   * Command to range and aim at the same time
   *
   * @return a double array composed of the aim velo then range velo
   */
  public double[] limelight_range_and_aim_proportional(List<Double> targets)
  {
    double[] velocities = new double[2];
    //Aim velocity[0] Range Velocity[1]
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kPAim = .0014; // .0017 for field relative command
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    if(!targets.contains(LimelightHelpers.getFiducialID("limelight")))
    {
      velocities[0] = 0;
      velocities[1] = 0;
      SmartDashboard.putBoolean("Limelight/ShootNow", false);
      SmartDashboard.putBoolean("Limelight/TargetIDDetected", false);
      return velocities;
    }
    SmartDashboard.putBoolean("Limelight/TargetIDDetected", true);

    velocities[0] = LimelightHelpers.getTX("limelight") * kPAim;

    SmartDashboard.putNumber("tartegtingAngularVelo", velocities[0]);

    // convert to radians per second for our drive method
    velocities[0] *=  6.283185307179586;

    //invert since tx is positive when the target is to the right of the crosshair
    velocities[0] *= -1.0;

    double kPRange = .0225; // .1 for field relative
    velocities[1] = LimelightHelpers.getTY("limelight") * kPRange;
    velocities[1] *= Constants.MAX_SPEED;
    velocities[1] *= 1.0;

    if(Math.abs(LimelightHelpers.getTX("limelight")) < 1.2 && 
      Math.abs(LimelightHelpers.getTY("limelight")) < 1.0 && 
      Math.abs(LimelightHelpers.getTX("limelight")) != 0.0)
    {
      SmartDashboard.putBoolean("Limelight/ShootNow", true);
    }
    else
    {
      SmartDashboard.putBoolean("Limelight/ShootNow", false);
    }

    return velocities;
  }

  public double[] limelight_red_amp_proposal(List<Double> targets)
  {
    double[] velocities = new double[3];
    //Aim velocity[0] Range Velocity[1]
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kPAim = .0014; // .0017 for field relative command
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    if(!targets.contains(LimelightHelpers.getFiducialID("limelight")))
    {
      velocities[0] = 0;
      velocities[1] = 0;
      velocities[2] = 0;
      SmartDashboard.putBoolean("Limelight/ShootNow", false);
      SmartDashboard.putBoolean("Limelight/TargetIDDetected", false);
      return velocities;
    }
    SmartDashboard.putBoolean("Limelight/TargetIDDetected", true);

    velocities[0] = (LimelightHelpers.getTX("limelight") - 7.0) * kPAim;

    // convert to radians per second for our drive method
    velocities[0] *=  Constants.MAX_SPEED;

    //invert since tx is positive when the target is to the right of the crosshair
    velocities[0] *= -1.0;

    double kPRange = .0225; // .1 for field relative
    velocities[1] = (LimelightHelpers.getTY("limelight") + 14.35) * kPRange;
    velocities[1] *= Constants.MAX_SPEED;
    velocities[1] *= 1.0;

    if(Math.abs(LimelightHelpers.getTX("limelight") - 7) < 1.2 && 
      Math.abs(LimelightHelpers.getTY("limelight")) < 1.0 && 
      Math.abs(LimelightHelpers.getTX("limelight")) != 0.0)
    {
      rotateToAlignWithAmp = true;
    }
    else
    {
      SmartDashboard.putBoolean("Limelight/ShootNow", false);
    }

    if(rotateToAlignWithAmp == true)
    {
      velocities[0] = (LimelightHelpers.getTX("limelight") - 36.75) * kPAim;

    // convert to radians per second for our drive method
    velocities[0] *=  6.283185307179586;

    //invert since tx is positive when the target is to the right of the crosshair
    velocities[0] *= -1.0; 
    velocities[1] = 0;
    velocities[2] = 1;
    SmartDashboard.putBoolean("Limelight/IsRotating", true);
    }
    else
    {
      SmartDashboard.putBoolean("Limelight/IsRotating", false);
    }
    SmartDashboard.putNumber("tartegtingAngularVelo", velocities[0]);
    return velocities;
  }

  public void resetAmpGate()
  {
    rotateToAlignWithAmp = false;
  }
    
}
