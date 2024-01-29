// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.commands.DriveCommand;
import frc.robot.commands.DriveFixedDistanceCommand;
import frc.robot.commands.test.RotationTestCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.RepeatCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  File jsonDirectory;

  CommandXboxController driverXbox = new CommandXboxController(0);

  private final SendableChooser<Command> autoChooser; 

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    // Configure the trigger bindings
    configureBindings();

    DriveCommand driveCmd = new DriveCommand(
      swerveSubsystem,
      () -> MathUtil.applyDeadband(-driverXbox.getLeftY(), 0.02), // Y axis on joystick is X axis for FRC. Forward is postive-Y, so need to invert sign
      () -> MathUtil.applyDeadband(-driverXbox.getLeftX(), 0.02), // X axis on joystick is Y axis for FRC. Left is positive-X, so need to invert sign
      () -> MathUtil.applyDeadband(-driverXbox.getRightX(), 0.02)); // Rotation for FRC is CCW-positive, so need to invert sign

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
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    // new Trigger(m_exampleSubsystem::exampleCondition)
    //     .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on rel 
    // m_driverController.b().whileTrue(m_exampleSubsystem.exampleMethodCommand());

    driverXbox.start().onTrue((new InstantCommand(swerveSubsystem::zeroGyro)));
    // driverXbox.x().whileTrue(new RepeatCommand(new InstantCommand(swerveSubsystem::moveVerySlowly)));
    // driverXbox.a().onTrue(new RotationTestCommand(swerveSubsystem));
    // driverXbox.b().onTrue(new SequentialCommandGroup(
    //   new DriveFixedDistanceCommand(swerveSubsystem, 1, 0, 1),
    //   new DriveFixedDistanceCommand(swerveSubsystem, 1, 90, 1),
    //   new DriveFixedDistanceCommand(swerveSubsystem, 1, 180, 1),
    //   new DriveFixedDistanceCommand(swerveSubsystem, 1, 270, 1)));
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