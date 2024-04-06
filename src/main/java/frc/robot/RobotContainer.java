// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.AutoAimCommand;
import frc.robot.commands.AutoArmCommand;
import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveWithAngleCommand;
import frc.robot.commands.DriveFixedDistanceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NudgeIntake;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinUpCommand;
import frc.robot.commands.TransferP1Command;
import frc.robot.commands.TransferP2Command;
import frc.robot.commands.test.RotationTestCommand;
import frc.robot.subsystems.AmpSubsystem;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.LEDSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.AmpSubsystem.AmpStates;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelDeadlineGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;
import java.util.Optional;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.hal.HALUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import static edu.wpi.first.units.Units.*;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  public final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(swerveSubsystem);
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  DoubleSubscriber visionTopic = inst.getDoubleTopic("/vision/right_joystick").subscribe(0.0);

  private final IntakeOuttakeSubsystem intake = new IntakeOuttakeSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();
  private final AmpSubsystem amp = new AmpSubsystem();
  private final LEDSubsystem led = new LEDSubsystem();

  File jsonDirectory;

  CommandXboxController driverXbox = new CommandXboxController(0);
  CommandXboxController operatorXbox = new CommandXboxController(1);

  private final SendableChooser<Command> autoChooser; 

  public Pose2d getPose() {
    return swerveSubsystem.getPose();
  }
  public double angleToSpeaker() {
    Optional<Alliance> ally = DriverStation.getAlliance();
    // SMELL: Using boolean short-circuiting to avoid throwing is very very cursed
    double speakerX = (ally.isPresent() && (ally.get() == Alliance.Blue)) ? 0.0 : 16.579342;
    Pose2d pose = swerveSubsystem.getPose();
    return Math.atan((pose.getY() - 5.547868) / (pose.getX() - speakerX));
  }

  // double invertIfRed(double value) {
  //   var alliance = DriverStation.getAlliance();
  //   var isRed = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  //   return 
  //   return MathUtil.applyDeadband(isRed ? driverXbox.getLeftY() : -driverXbox.getLeftY(), 0.02); // Y axis on joystick is X axis for FRC. Forward is postive-Y, so need to invert sign
  // }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("intake", new ParallelCommandGroup(
      arm.runPIDwithAngle(ArmStates.INTAKE),
      new IntakeCommand(intake, true, true)));
    NamedCommands.registerCommand("shoot", new SequentialCommandGroup(
      new ParallelDeadlineGroup(
        new WaitCommand(.75),
        new InstantCommand(() -> intake.setMotors(0, Constants.Speeds.OUTTAKE_SPEED)),
        new AutoArmCommand(arm, () -> swerveSubsystem.getPose().getX(), () -> swerveSubsystem.getPose().getY()),
        new DriveCommand(swerveSubsystem, () -> 0.0,() -> 0.0,() -> 0.0,() -> true,() -> angleToSpeaker())
      ),
      new InstantCommand(() -> intake.setMotors(-100, Constants.Speeds.OUTTAKE_SPEED)),
      new WaitCommand(.2)
    ));
    NamedCommands.registerCommand("spinUp", new InstantCommand(() -> intake.setMotors(0, Constants.Speeds.OUTTAKE_SPEED)));
    NamedCommands.registerCommand("stow", arm.setAngleCommand(ArmStates.STOW));
    NamedCommands.registerCommand("intakePosition", arm.setAngleCommand(ArmStates.INTAKE));
    NamedCommands.registerCommand("stopCommand", new InstantCommand(() -> intake.setMotors(0, 0)));
    NamedCommands.registerCommand("zeroGyro", new InstantCommand(() -> swerveSubsystem.setGyroOffset()));

    // Configure the trigger bindings
    configureBindings();

   DriveCommand driveCmd = new DriveCommand(
      swerveSubsystem,
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.02), // Y axis on joystick is X axis for FRC. Forward is postive-Y, so need to invert sign
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.02), // X axis on joystick is Y axis for FRC. Left is positive-X, so need to invert sign
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.08),
      () -> driverXbox.getHID().getBButton(),
      () -> angleToSpeaker()
    ); // Rotation for FRC is CCW-positive, so need to invert sign

    arm.setDefaultCommand(arm.runPIDCommand());
    amp.setDefaultCommand(amp.runPIDCommand());
      
    swerveSubsystem.setDefaultCommand(driveCmd);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Path", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via them
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    driverXbox.start().onTrue((new InstantCommand(swerveSubsystem::zeroGyro)));
    driverXbox.y().onTrue(arm.setAngleCommand(120.0));
    driverXbox.a().onTrue(arm.setAngleCommand(ArmStates.STOW));
    driverXbox.x().whileTrue(intake.ejectCommand());
    driverXbox.x().onFalse(intake.stopCommand());
    operatorXbox.x().onTrue(amp.runPIDCommand(AmpStates.STOW));
    // #region testing commands
    // // driverXbox.x().whileTrue(new RepeatCommand(new InstantCommand(swerveSubsystem::moveVerySlowly)));
    // driverXbox.b().onTrue(new SequentialCommandGroup(
    //   new IntakeCommand(intake),
    //   new InstantCommand(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.5)),
    //   new WaitCommand(0.5),
    //   new InstantCommand(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0)),
    //   new SpinUpCommand(intake)
    // ));
    // driverXbox.rightTrigger().onTrue(new ShootCommand(intake));
    // driverXbox.a().onTrue(new RotationTestCommand(swerveSubsystem));
    // driverXbox.b().onTrue(new SequentialCommandGroup(
    //   // new DriveFixedDistanceCommand(swerveSubsystem, 1, 0, 1),
    //   // new DriveFixedDistanceCommand(swerveSubsystem, 1, 90, 1),
    //   // new DriveFixedDistanceCommand(swerveSubsystem, 1, 180, 1),
    //   // new DriveFixedDistanceCommand(swerveSubsystem, 1, 270, 1)));

    // Nudge the intake
    // driverXbox.y().onTrue(Commands.race(new NudgeIntake(intake), new WaitCommand(0.2)));
    // #endregion testing commands
    
    // DANNY CONTROLS
    // Sticks for movement and rotation - this is already handled by the library

    // Right trigger to intake - lower arm, spin
    driverXbox.leftTrigger().onTrue(new SequentialCommandGroup(
        arm.setAngleCommand(ArmStates.INTAKE),
        new IntakeCommand(intake, false, true),
        // new ParallelDeadlineGroup(new WaitCommand(.5), new InstantCommand(() -> intake.setMotors(Constants.Speeds.INTAKE_SPEED, -200))),
        // new IntakeCommand(intake, false, false), // Simultaneously lower the arm and start the intake. Once the IntakeCommand is done (ie we have a note)...
        // new ParallelDeadlineGroup(new WaitCommand(.1), new InstantCommand(() -> intake.setMotors(.35, -200))),
        // new InstantCommand(() -> intake.setMotors(Constants.Speeds.INTAKE_SPEED, -200)),
        // new WaitCommand(.2),
        // new IntakeCommand(intake, false, false), // Simultaneously lower the arm and start the intake. Once the IntakeCommand is done (ie we have a note)...
        // new ParallelDeadlineGroup(new WaitCommand(.1), new InstantCommand(() -> intake.setMotors(.35, -200))),
        // new InstantCommand(() -> intake.setMotors(0, -200)),
        arm.setAngleCommand(ArmStates.STOW),
        new InstantCommand(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.5)), // Rumble the controller
        new WaitCommand(0.5), // Wait half a second
        new InstantCommand(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0)), // Stop rumbling
        intake.startOutake())
        ); 
    // B to shootw
    driverXbox.rightTrigger().onTrue(new SequentialCommandGroup(
      new ShootCommand(intake)
      // new WaitCommand(1),
      // arm.setAngleCommand(ArmStates.STOW)
    ));
    // Left trigger to move to shooting position
    // driverXbox.b().onTrue(new ParallelCommandGroup(new AutoAimCommand(arm, intake, () -> swerveSubsystem.getPose().getX(), () -> swerveSubsystem.getPose().getY())));
    // SmartDashboard.putNumber("armSetpointTest", 75.0);
    // driverXbox.b().onTrue(new ParallelCommandGroup(intake.setMotors(0, () -> SmartDashboard.getNumber("shooterRPM", 1000)), arm.setAngleCommand(() -> SmartDashboard.getNumber("armSetpointTest", 75.0))));
    driverXbox.b().whileTrue(new SequentialCommandGroup(intake.startOutake(), new AutoArmCommand(arm, () -> swerveSubsystem.getPose().getX(), () -> swerveSubsystem.getPose().getY())));
    // SmartDashboard.putNumber("setpoint2", 75);
    // driverXbox.b().whileTrue(new ParallelCommandGroup(intake.startOutake(), arm.setAngleCommand(() -> SmartDashboard.getNumber("setpoint2", 30))));

    // Left bumper moves to stowed position
    // driverXbox.leftBumper().onTrue(arm.setAngleCommand(ArmStates.STOW));
    // Right bumper stops intake
    driverXbox.rightBumper().onTrue(new ParallelCommandGroup(
      intake.stopCommand(),
      new InstantCommand(() -> amp.motor1.set(0))
    ));

    operatorXbox.rightBumper().onTrue(new ParallelCommandGroup(
      intake.stopCommand(),
      new InstantCommand(() -> amp.motor1.set(0))
    ));

    operatorXbox.b().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> intake.setMotors(0.0, 0.0)),
      new WaitCommand(.25),
      arm.setAngleCommand(ArmStates.INTAKE),
      new WaitCommand(.2),
      amp.runPIDCommand(AmpStates.TRANSFER),
      arm.runPIDwithAngle(ArmStates.TRANSFER),
      new WaitCommand(1),
      new TransferP1Command(intake, amp),
      new TransferP2Command(intake, amp)
    ));

    operatorXbox.leftTrigger().onTrue(amp.runPIDCommand(AmpStates.SHOOT));

    operatorXbox.rightTrigger().onTrue(new SequentialCommandGroup(
      new InstantCommand(() -> amp.motor1.set(-.6)),
      new WaitCommand(1),
      new InstantCommand(() -> amp.motor1.set(0)),
      amp.runPIDCommand(AmpStates.STOW)
    ));

    // driverXbox.leftBumper().onTrue(intake.startOutake())

    // Reset pose-estimation when starting auton
    RobotModeTriggers.autonomous().onTrue(new InstantCommand(() -> {swerveSubsystem.resetGyroTo(swerveSubsystem.getPose().getRotation());}));
    // RobotModeTriggers.autonomous().onTrue(new SequentialCommandGroup(
    //   amp.runPIDCommand(AmpStates.UNLOAD),
    //   new WaitCommand(.1),
    //   arm.runPIDwithAngle(ArmStates.INTAKE),
    //   new WaitCommand(.5),
    //   amp.runPIDCommand(AmpStates.STOW)
    // ));
    // Brake disabling
    // Automatically go back to brake when you enable
    RobotModeTriggers.disabled().onFalse(arm.setBrakeCommand(true));
    // Disable brake when user button pressed
    new Trigger(HALUtil::getFPGAButton).onTrue(arm.setBrakeCommand(false));

    // Stop rumble on disable
    RobotModeTriggers.disabled().onTrue(new InstantCommand(() -> {driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0);}).ignoringDisable(true));

    // #region LED commands
    new Trigger(intake::intakeIsStalled).whileTrue(led.indicateIntookCommand());
    new Trigger(intake::intakeIsStalled).onFalse(led.setAnimationToAllianceColorCommand(DriverStation.getAlliance()));
    operatorXbox.leftBumper().onTrue(led.indicateNeedNoteCommand());
    operatorXbox.rightBumper().onTrue(led.setAnimationToAllianceColorCommand(DriverStation.getAlliance()));
    // #endregion
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return autoChooser.getSelected();
  }
}