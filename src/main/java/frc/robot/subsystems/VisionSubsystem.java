package frc.robot.subsystems;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.utils.LimelightHelpers;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
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
    InterpolatingDoubleTreeMap distanceMap;
    public VisionSubsystem()
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");
        tx = table.getEntry("tx");
        ty = table.getEntry("ty");
        ta = table.getEntry("ta");
        distanceMap = new InterpolatingDoubleTreeMap();
        //DistanceMap will use linear interpolation to quickly determine the arm position with enough known shooting positions
        //The first value will be the distance in inches and the output will be the corresponding anlge in degrees.

        //TODO: CURRENTLY THE DISTANCE VALUE IS UNKNOWN, we probably want one or two more values, 2 shoots between pod and sub probably
        //Subshoot
        distanceMap.put(48.57, 53.0);
        //Limelight autodistance shot (podshot)
        distanceMap.put(109.7, 35.5);
        //Mid long shot
        distanceMap.put(168.0, 31.0);
        //Long Range Shot
        distanceMap.put(215.35, 27.25);

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
    double kP = .0014; // .025
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

  public double[] limelight_amp_proposal(List<Double> targets)
  {
    double[] velocities = new double[3];
    //[X, Y, rot] order in array
    // kP (constant of proportionality)
    // this is a hand-tuned number that determines the aggressiveness of our proportional control loop
    // if it is too high, the robot will oscillate around.
    // if it is too low, the robot will never reach its target
    // if the robot never turns in the correct direction, kP should be inverted.
    double kPTranslateX = .0225;
    double kPTranslateY = .0225;
    double kPAim = .0014;
    // tx ranges from (-hfov/2) to (hfov/2) in degrees. If your target is on the rightmost edge of 
    // your limelight 3 feed, tx should return roughly 31 degrees.
    if(!targets.contains(LimelightHelpers.getFiducialID("limelight")))
    {
      velocities[0] = 0;
      velocities[1] = 0;
      velocities[2] = 0;
      return velocities;
    }
    SmartDashboard.putBoolean("Limelight/TargetIDDetected", true);

    Pose3d cameraPose = LimelightHelpers.getTargetPose3d_CameraSpace("limelight");
    if(DriverStation.getAlliance().get() == DriverStation.Alliance.Red)
    {
      velocities[0] = (cameraPose.getX() - 2.7) * kPTranslateX;
      velocities[0] *=  Constants.MAX_SPEED;
      velocities[0] *= -1.0;

      velocities[1] = (cameraPose.getY() - .73) * kPTranslateY;
      velocities[1] *= Constants.MAX_SPEED;
      velocities[1] *= 1.0;

      velocities[2] = (cameraPose.getRotation().getZ() + 19) * kPAim;
      velocities[2] *=  6.283185307179586;
      velocities[2] *= -1.0;
    }
    else
    {
      velocities[0] = (cameraPose.getX() - 7.0) * kPTranslateX;
      velocities[0] *=  Constants.MAX_SPEED;
      velocities[0] *= -1.0;

      velocities[1] = (cameraPose.getY() + 14.35) * kPTranslateY;
      velocities[1] *= Constants.MAX_SPEED;
      velocities[1] *= 1.0;

      velocities[2] = (cameraPose.getRotation().getZ()) * kPAim;
      velocities[2] *=  6.283185307179586;
      velocities[2] *= -1.0;
    }
    
    
    return velocities;
  }

  public double limelight_arm_aim_0to1(List<Double> targets)
  {
    SmartDashboard.putNumber("Limelight/armSetpoint", 0.0);
    if(!targets.contains(LimelightHelpers.getFiducialID("limelight")))
    {
      SmartDashboard.putNumber("Limelight/armSetpoint", .125);
      return 0.125;
    }
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
    NetworkTableEntry ty = table.getEntry("ty");
    double targetOffsetAngle_Vertical = ty.getDouble(0.0);

    // how many degrees back is your limelight rotated from perfectly vertical?
    double limelightMountAngleDegrees = 22.323; 

    // distance from the center of the Limelight lens to the floor
    double limelightLensHeightInches = 6.963936; 

    // distance from the target to the floor, the height of the april tag: https://firstfrc.blob.core.windows.net/frc2024/FieldAssets/2024LayoutMarkingDiagram.pdf
    double goalHeightInches = 57.125; 

    double angleToGoalDegrees = limelightMountAngleDegrees + targetOffsetAngle_Vertical;
    double angleToGoalRadians = angleToGoalDegrees * (3.14159 / 180.0);

    //calculate distance
    double distanceFromLimelightToGoalInches = (goalHeightInches - limelightLensHeightInches) / Math.tan(angleToGoalRadians);

    SmartDashboard.putNumber("Limelight/DistanceFromGoal", distanceFromLimelightToGoalInches);

    //TODO: get measurement with robot against the subwoofer, the measurement with the robot in our robot's typical shooting spot and find a further out shot that we can hit (or we can use that range as our max distance).
    // The goal is to fit these measurements to the angles required at know setpoints then fitting it to a linear function
    // We will use an interpolatingDoubleTreeMap for this, look at the visionSubsystem constructor for more info

    //distanceMap.get() will linearly inpterpet the correct angle, and we clamp it between acceptable measurements for the arm position
    double armSetpoint = MathUtil.clamp(distanceMap.get(distanceFromLimelightToGoalInches)/360, 0.05, 0.1527);

    SmartDashboard.putNumber("Limelight/armSetpoint", armSetpoint);

    return armSetpoint;
  }

  public void resetAmpGate()
  {
    rotateToAlignWithAmp = false;
  }
    
}
