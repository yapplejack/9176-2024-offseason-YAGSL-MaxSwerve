// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandJoystick;
import edu.wpi.first.wpilibj2.command.button.CommandPS4Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.VisionConstants;
import frc.robot.commands.arm.ArmToPosition;
import frc.robot.commands.autoCommands.AimSubThenPod;
import frc.robot.commands.autoCommands.ArmPositionAuto;
import frc.robot.commands.autoCommands.PodShotB;
import frc.robot.commands.autoCommands.StopMotors;
import frc.robot.commands.autoCommands.SubshotB;
import frc.robot.commands.backpack.RunBackpack;
import frc.robot.commands.climb.RunClimb;
import frc.robot.commands.climb.RunClimbLeftDown;
import frc.robot.commands.climb.RunClimbLeftUp;
import frc.robot.commands.descend.RunDescend;
import frc.robot.commands.feeder.ReverseFeeder;
import frc.robot.commands.feeder.RunFeeder;
import frc.robot.commands.indexer.RunIndexerShooter;
import frc.robot.commands.intake.RunIntakeUnjam;
import frc.robot.commands.manipCommands.intakeFromFloor;
import frc.robot.commands.manipCommands.intakeFromFloorAmp;
import frc.robot.commands.manipCommands.intakeFromSource;
import frc.robot.commands.manipCommands.transferToAmpback;
import frc.robot.commands.manipCommands.transferToShooter;
import frc.robot.commands.shooter.PassShooter;
import frc.robot.commands.shooter.RunShooter;
import frc.robot.commands.swervedrive.drivebase.AbsoluteDrive;
import frc.robot.commands.vision.aimTele;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.BackpackSubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.FeederSubsystem;
import frc.robot.subsystems.IndexerSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import frc.robot.subsystems.ArmSubsystem.armPositions;

import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{

  // Replace with CommandPS4Controller or CommandJoystick if needed
  public final CommandPS4Controller m_driverController = new CommandPS4Controller(0);

  public final CommandJoystick m_manipController = new CommandJoystick(1);
  // The robot's subsystems and commands are defined here...
  final SwerveSubsystem drivebase = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(),
                                                                         "swerve"));

  final VisionSubsystem m_vision = new VisionSubsystem();
   public final ArmSubsystem m_arm = new ArmSubsystem();
  public final BackpackSubsystem m_backpack = new BackpackSubsystem();
  public final ClimbSubsystem m_climb = new ClimbSubsystem();
  public final FeederSubsystem m_feeder = new FeederSubsystem();
  public final ShooterSubsystem m_shooter = new ShooterSubsystem();
  public final IndexerSubsystem m_indexer = new IndexerSubsystem();
  public final IntakeSubsystem m_intake = new IntakeSubsystem();

  private final SendableChooser<Command> autoChooser;

  private Field2d m_field = new Field2d();

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    configureBindings();

    NamedCommands.registerCommand("SubShotB", new SubshotB(m_arm, m_indexer, m_shooter));
    NamedCommands.registerCommand("PodShotB", new PodShotB(m_arm, m_indexer, m_shooter));
    NamedCommands.registerCommand("StopMotors", new StopMotors(m_indexer, m_shooter));
    NamedCommands.registerCommand("Intake", new intakeFromFloor(m_intake, m_feeder, m_indexer));
    NamedCommands.registerCommand("ArmToPositionIntake", new ArmPositionAuto(m_arm, armPositions.INTAKE));
    NamedCommands.registerCommand("OverAim", new AimSubThenPod(m_arm, armPositions.SUBSHOT));
    //NamedCommands.registerCommand("Aim", new AimVision(m_robotDrive, m_vision));


    // Build an auto chooser. This will use Commands.none() as the default option.
    autoChooser = AutoBuilder.buildAutoChooser();

    // Another option that allows you to specify the default auto by its name
    // autoChooser = AutoBuilder.buildAutoChooser("My Default Auto");

    SmartDashboard.putData("Auto Chooser", autoChooser);

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the desired angle NOT angular rotation
    Command driveFieldOrientedVelo = drivebase.driveCommand(() -> MathUtil.applyDeadband(m_driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND), 
    () -> MathUtil.applyDeadband(m_driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND), 
    () -> MathUtil.applyDeadband(-m_driverController.getRightX(), OperatorConstants.RIGHT_X_DEADBAND));

    Command driveFieldOrintedAngle = drivebase.driveCommand(() -> m_driverController.getRawAxis(1), 
    () -> m_driverController.getRawAxis(0), 
    () -> -m_driverController.getRightX(),
    () -> -m_driverController.getRightY());

    // Applies deadbands and inverts controls because joysticks
    // are back-right positive while robot
    // controls are front-left positive
    // left stick controls translation
    // right stick controls the angular velocity of the robot
    Command driveFieldOrientedVeloSim = drivebase.driveCommand(() -> m_driverController.getRawAxis(1), 
    () -> m_driverController.getRawAxis(0), 
    () -> -m_driverController.getRightX());

        Command driveFieldOrientedDirectAngleSim = drivebase.simDriveCommand(
        () -> MathUtil.applyDeadband(m_driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
        () -> MathUtil.applyDeadband(m_driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
        () -> m_driverController.getRightX(),
        () -> m_driverController.getLeftY());

    drivebase.setDefaultCommand(
      !RobotBase.isSimulation() ? driveFieldOrientedVelo : driveFieldOrientedVeloSim);
    
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
    //return new PathPlannerAuto("startB-shoot2");
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`

    m_driverController.button(1).onTrue((Commands.runOnce(drivebase::zeroGyro)));
    //driverXbox.button(2).onTrue(Commands.runOnce(drivebase::addFakeVisionReading));
    //driverXbox.button(3).onTrue((Commands.runOnce(drivebase::lock)));

    //m_driverController.button(2).whileTrue(new RunCommand(() -> drivebase.driveCommand(
    //  () -> MathUtil.applyDeadband(m_driverController.getRawAxis(1), OperatorConstants.LEFT_Y_DEADBAND),
    //  () -> MathUtil.applyDeadband(m_driverController.getRawAxis(0), OperatorConstants.LEFT_Y_DEADBAND),
    //  () -> m_vision.limelight_aim_proportional()),
    //  drivebase));
    
    /*m_driverController.button(4).whileTrue(new aimTele(drivebase, m_vision, 
      MathUtil.applyDeadband(m_driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND), VisionConstants.speakerTargets));*/

      m_driverController.button(4).whileTrue(new RunCommand(() -> 
      drivebase.aimAtSpeaker(MathUtil.applyDeadband(m_driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
       m_vision.limelight_range_and_aim_proportional(VisionConstants.speakerTargets)), drivebase));

    /*m_driverController.button(3).whileTrue(new ParallelCommandGroup( new aimTele(drivebase, m_vision, 
      MathUtil.applyDeadband(m_driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND), VisionConstants.passingTargets), 
      new ArmToPosition(m_arm, armPositions.PASS), new PassShooter(m_shooter) ));*/

    m_driverController.button(3).whileTrue(new ParallelCommandGroup( new RunCommand(() -> 
      drivebase.aimAtSpeaker(MathUtil.applyDeadband(m_driverController.getRawAxis(0), OperatorConstants.LEFT_X_DEADBAND),
       m_vision.limelight_range_and_aim_proportional(VisionConstants.speakerTargets)), drivebase), 
      new ArmToPosition(m_arm, armPositions.PASS), new PassShooter(m_shooter) ));

    m_driverController.axisGreaterThan(3, -.5).whileTrue(new transferToAmpback(m_feeder, m_indexer));
    m_driverController.axisGreaterThan(4, -.5).onTrue(new transferToShooter(m_feeder, m_indexer));
    //m_manipController.button(5).whileTrue(new manipIntake(m_arm, m_intake, m_feeder, m_indexer));
    //m_manipController.button(6).whileTrue(new stowArm(m_arm));

    //m_manipController.button(4).onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.INTAKE));
    //m_manipController.button(2).onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.AMP));
    //m_manipController.button(1).onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.STOWED));
    //m_manipController.button(3).onTrue(new ArmToPosition(m_arm, ArmSubsystem.armPositions.SOURCE));
    //Intake Note
    m_driverController.R1().whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.INTAKE), new intakeFromFloor(m_intake, m_feeder, m_indexer))).onFalse(new ArmToPosition(m_arm, armPositions.INTAKE));
    //Stow arm
    m_driverController.L1().onTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.INTAKE), new intakeFromFloorAmp(m_intake, m_feeder, m_indexer))).onFalse(new ArmToPosition(m_arm, armPositions.INTAKE));

    m_driverController.button(12).onTrue(new ArmToPosition(m_arm, armPositions.STOWED));
    //m_driverController.button(2).whileTrue(new findColor(m_indexer));

    //Intake from source
    m_manipController.button(1).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.SOURCE), new intakeFromSource(m_shooter, m_indexer))).onFalse(new ArmToPosition(m_arm, armPositions.SOURCE));
    //Stow arm
    //m_manipController.button(5).onTrue(new ArmToPosition(m_arm, armPositions.INTAKE)).onFalse(new ArmToPosition(m_arm, armPositions.STOWED));
    //Shoot into speaker from sub
    m_manipController.button(2).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.SUBSHOT), 
    new RunShooter(m_shooter))).onFalse(new ArmToPosition(m_arm, armPositions.SUBSHOT));//.and(m_manipController.button(6)).whileTrue(new SequentialCommandGroup
    //(new ReverseFeeder(m_feeder), new ParallelCommandGroup(new RunFeeder(m_feeder), new RunIndexerShooter(m_indexer))));

    m_manipController.button(3).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.PODSHOT), new RunShooter(m_shooter))).onFalse(new ArmToPosition(m_arm, armPositions.PODSHOT));
    //Shoot into speaker from pod
    //m_manipController.button(3).whileTrue(new ParallelCommandGroup(new ArmToPosition(m_arm, armPositions.PODSHOT), 
    //new RunShooter(m_shooter))).and(m_manipController.button(6).whileTrue(new SequentialCommandGroup
    //(new ReverseFeeder(m_feeder), new ParallelCommandGroup(new RunFeeder(m_feeder), new RunIndexerShooter(m_indexer)))));

    m_manipController.button(6).whileTrue(new ParallelCommandGroup(new RunFeeder(m_feeder), new RunIndexerShooter(m_indexer)));

    //m_manipController.button(7).whileTrue(new RunClimbRightDown(m_climb));
    m_manipController.button(7).whileTrue(new RunClimb(m_climb));

    m_manipController.button(8).whileTrue(new RunDescend(m_climb));

    //m_manipController.button(8).whileTrue(new RunClimbRightUp(m_climb));

    m_manipController.button(9).whileTrue(new RunClimbLeftDown(m_climb));

    m_manipController.button(10).whileTrue(new RunClimbLeftUp(m_climb));

    //Unjam deadzone
    m_manipController.pov(0).whileTrue(new ParallelCommandGroup(new RunIntakeUnjam(m_intake), new ArmToPosition(m_arm, armPositions.INTAKE), new ReverseFeeder(m_feeder))).onFalse(new ArmToPosition(m_arm, armPositions.INTAKE));
    //Unjam intake
    m_manipController.pov(90).whileTrue(new ParallelCommandGroup(new RunIntakeUnjam(m_intake), new ArmToPosition(m_arm, armPositions.INTAKE), new ReverseFeeder(m_feeder))).onFalse(new ArmToPosition(m_arm, armPositions.INTAKE));
    //Unjam  shooter
    m_manipController.pov(180).whileTrue(new ParallelCommandGroup(new intakeFromFloorAmp(m_intake, m_feeder, m_indexer), new ArmToPosition(m_arm, armPositions.INTAKE))).onFalse(new ArmToPosition(m_arm, armPositions.INTAKE));

    m_manipController.pov(270).whileTrue(new ParallelCommandGroup(new intakeFromFloor(m_intake, m_feeder, m_indexer), new ArmToPosition(m_arm, armPositions.INTAKE))).onFalse(new ArmToPosition(m_arm, armPositions.INTAKE));

    m_manipController.button(5).whileTrue(new RunBackpack(m_backpack, m_indexer));

    m_manipController.button(4).whileTrue(new ArmToPosition(m_arm, armPositions.AMP)).onFalse(new ArmToPosition(m_arm, armPositions.AMP));
    /* 
    driverXbox.b().whileTrue(
        Commands.deferredProxy(() -> drivebase.driveToPose(
                                   new Pose2d(new Translation2d(4, 4), Rotation2d.fromDegrees(0)))
                              ));*/
    // driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  //public Command getAutonomousCommand()
 // {
    // An example command will be run in autonomous
 //   return drivebase.getAutonomousCommand("New Auto");
 // }

  public void setDriveMode()
  {
    //drivebase.setDefaultCommand();
  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }
}