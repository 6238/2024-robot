// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveFixedDistanceCommand;
import frc.robot.commands.IntakeCommand;
import frc.robot.commands.NudgeIntake;
import frc.robot.commands.ShootCommand;
import frc.robot.commands.SpinUpCommand;
import frc.robot.commands.test.RotationTestCommand;
import frc.robot.subsystems.ArmSubsystem;
import frc.robot.subsystems.IntakeOuttakeSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.subsystems.ArmSubsystem.ArmStates;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));
  private final VisionSubsystem visionSubsystem = new VisionSubsystem(Constants.PHOTONCAMERA_NAME, swerveSubsystem);
  NetworkTableInstance inst = NetworkTableInstance.getDefault();
  DoubleSubscriber visionTopic = inst.getDoubleTopic("/vision/right_joystick").subscribe(0.0);

  private final IntakeOuttakeSubsystem intake = new IntakeOuttakeSubsystem();
  private final ArmSubsystem arm = new ArmSubsystem();

  File jsonDirectory;

  CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser; 

  // double invertIfRed(double value) {
  //   var alliance = DriverStation.getAlliance();
  //   var isRed = alliance.isPresent() ? alliance.get() == DriverStation.Alliance.Red : false;
  //   return 
  //   return MathUtil.applyDeadband(isRed ? driverXbox.getLeftY() : -driverXbox.getLeftY(), 0.02); // Y axis on joystick is X axis for FRC. Forward is postive-Y, so need to invert sign
  // }

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    NamedCommands.registerCommand("setAngleIntake", arm.setAngleCommand(ArmStates.INTAKE));
    NamedCommands.registerCommand("setAngleShoot", arm.setAngleCommand(ArmStates.SHOOT));
    NamedCommands.registerCommand("setAngleStow", arm.setAngleCommand(ArmStates.STOW));
    NamedCommands.registerCommand("IntakeCommand", new IntakeCommand(intake));
    NamedCommands.registerCommand("SpinUpCommand", intake.startOutake());
    NamedCommands.registerCommand("ShootCommand", new ShootCommand(intake));
    NamedCommands.registerCommand("stopCommand", intake.stopCommand());

    NamedCommands.registerCommand("zeroRight", new InstantCommand(() ->
      swerveSubsystem.setGyroOffset(Degrees.of(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 + 60 : 60).in(Radians)))
    );
    NamedCommands.registerCommand("zeroCenter", new InstantCommand(() ->
      swerveSubsystem.setGyroOffset(Degrees.of(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 : 0).in(Radians)))
    );
    NamedCommands.registerCommand("zeroLeft", new InstantCommand(() ->
      swerveSubsystem.setGyroOffset(Degrees.of(DriverStation.getAlliance().get() == DriverStation.Alliance.Red ? 180 - 60 : -60).in(Radians)))
    );
    NamedCommands.registerCommand("reZero", new InstantCommand(() ->
      swerveSubsystem.setGyroOffset(Degrees.of(swerveSubsystem.getPose().getRotation().getDegrees() - 180).in(Radians)))
    );
    // Configure the trigger bindings
    configureBindings();

   DriveCommand driveCmd = new DriveCommand(
      swerveSubsystem,
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.08),
      () -> driverXbox.getHID().getBButton(),
      () -> -Math.atan((swerveSubsystem.getPose().getY() - 5.547868) / swerveSubsystem.getPose().getX())
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.02), // X axis on joystick is Y axis for FRC. Left is positive-X, so need to invert sign
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.02), // Y axis on joystick is X axis for FRC. Forward is postive-Y, so need to invert sign
    ); // Rotation for FRC is CCW-positive, so need to invert sign

    arm.setDefaultCommand(arm.runPIDCommand());
      
    swerveSubsystem.setDefaultCommand(driveCmd);

    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Path", autoChooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    driverXbox.start().onTrue((new InstantCommand(swerveSubsystem::zeroGyro)));
    driverXbox.y().onTrue(arm.setAngleCommand(100.0));
    driverXbox.a().onTrue(arm.setAngleCommand(ArmStates.STOW));
    driverXbox.x().whileTrue(intake.ejectCommand());
    driverXbox.x().onFalse(intake.stopCommand());
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
        new ParallelCommandGroup(arm.setAngleCommand(ArmStates.INTAKE), new IntakeCommand(intake)), // Simultaneously lower the arm and start the intake. Once the IntakeCommand is done (ie we have a note)...
        arm.setAngleCommand(ArmStates.STOW),
        new InstantCommand(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0.5)), // Rumble the controller
        new WaitCommand(0.5), // Wait half a second
        new InstantCommand(() -> driverXbox.getHID().setRumble(RumbleType.kBothRumble, 0))) // Stop rumbling
        ); 
    // B to shoot
    driverXbox.rightTrigger().onTrue(new SequentialCommandGroup(
      new ShootCommand(intake),
      arm.setAngleCommand(ArmStates.STOW)
    ));
    // Left trigger to move to shooting position
    driverXbox.b().onTrue(new ParallelCommandGroup(arm.setAngleCommand(ArmStates.SHOOT), new SpinUpCommand(intake)));
    // Left bumper moves to stowed position
    // driverXbox.leftBumper().onTrue(arm.setAngleCommand(ArmStates.STOW));
    // Right bumper stops intake
    driverXbox.rightBumper().onTrue(intake.stopCommand());

    // Reset pose-estimation when starting auton
    RobotModeTriggers.autonomous().onTrue(new InstantCommand(() -> {swerveSubsystem.resetGyroTo(swerveSubsystem.getPose().getRotation());}));
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