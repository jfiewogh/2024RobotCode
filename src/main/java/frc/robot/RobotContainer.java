// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.util.datalog.StringLogEntry;

import com.ctre.phoenix6.swerve.SwerveModule;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import frc.robot.commands.SwerveTeleopCommand;
import frc.robot.subsystems.BatterySubsystem;
import frc.robot.subsystems.ClimbSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.commands.ShooterAutonCommand;
import frc.robot.commands.ShooterCommand;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.BlinkinLedSubsystem.BlinkinMode;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.BlinkinLedSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.StartEndCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ClimbPosition;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem swerveSubsystem;
  private final ShooterSubsystem shooterSubsystem;
  private final ShooterCommand shooterCommand;
  private final ShooterAutonCommand shooterAutonCommand;
  private final IntakeSubsystem intakeSubsystem;
  private final BatterySubsystem batterySubsystem;
  private final ClimbSubsystem climbSubsystem;
  private final BlinkinLedSubsystem blinkinSubsystem;
  private ShuffleboardTab configTab = Shuffleboard.getTab("config");
  private SendableChooser<Command> autonChooser = null;
  private DataLog robotSubsystemsLog = DataLogManager.getLog();
  private StringLogEntry subsystemEnabledLog = new StringLogEntry(robotSubsystemsLog, "/Subsystems Enabled/"); 
  private final SendableChooser<Command> climbArmModeChooser = new SendableChooser<>();

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  @SuppressWarnings("unused")
  public RobotContainer() {
    blinkinSubsystem = Robot.blinkinEnabled ? new BlinkinLedSubsystem() : null;

    swerveSubsystem = Robot.swerveEnabled ? new SwerveSubsystem() : null;
    subsystemEnabledLog.append(swerveSubsystem == null ? "Swerve: Disabled" : "Swerve: Enabled");

    shooterSubsystem = Robot.shooterEnabled ? new ShooterSubsystem() : null;
    subsystemEnabledLog.append(swerveSubsystem == null ? "Shooter: Disabled" : "Shooter: Enabled");

    intakeSubsystem = Robot.intakeEnabled ? new IntakeSubsystem() : null;
    subsystemEnabledLog.append(intakeSubsystem == null ? "Intake: Disabled" : "Intake: Enabled");

    batterySubsystem = Robot.batteryEnabled ? new BatterySubsystem() : null;
    subsystemEnabledLog.append(batterySubsystem == null ? "Battery Disabled" : "Battery Enabled");
    
    climbSubsystem = Robot.climbEnabled ? new ClimbSubsystem() : null;
    subsystemEnabledLog.append(climbSubsystem == null ? "Climb: Disabled" : "Climb: Enabled");
    
    shooterCommand = (Robot.shooterEnabled && Robot.intakeEnabled) ? new ShooterCommand(intakeSubsystem, shooterSubsystem, blinkinSubsystem, blinkinSubsystem != null) : null;
    shooterAutonCommand = (Robot.shooterEnabled && Robot.intakeEnabled) ? new ShooterAutonCommand(intakeSubsystem, shooterSubsystem, blinkinSubsystem, blinkinSubsystem != null) : null;

    if (Robot.climbEnabled) {
      climbArmModeChooser.setDefaultOption("Uniform", new InstantCommand(
        () -> climbSubsystem.toggleClimb(ClimbPosition.Uniform),
        climbSubsystem)
      );
      climbArmModeChooser.addOption("Left Elevated", new InstantCommand(
        () -> climbSubsystem.toggleClimb(ClimbPosition.LeftElevated),
        climbSubsystem)
      );
      climbArmModeChooser.addOption("Right Elevated", new InstantCommand(
        () -> climbSubsystem.toggleClimb(ClimbPosition.RightElevated),
        climbSubsystem)
      );

      SmartDashboard.putData(climbArmModeChooser);
    }

    if (Robot.swerveEnabled) {
      // autonChooser = AutoBuilder.buildAutoChooser("Center_2Note");
      // configTab.add("Auton Selection", autonChooser).withSize(3, 1);

      swerveSubsystem.setDefaultCommand(new SwerveTeleopCommand(
        swerveSubsystem, 
        OI.Driver.getXTranslationSupplier(),
        OI.Driver.getYTranslationSupplier(),
        OI.Driver.getRotationSupplier())
      );
    }

    if (Robot.blinkinEnabled) {
      blinkinSubsystem.setDefaultCommand(new InstantCommand(() -> blinkinSubsystem.setMode(BlinkinMode.Default), blinkinSubsystem));
    }

    if (Robot.intakeEnabled){
      NamedCommands.registerCommand(
        "Intake", 
        new InstantCommand(() -> intakeSubsystem.setIntakePosition("deploy"), intakeSubsystem)
          .andThen(new InstantCommand(() -> intakeSubsystem.runIntakeMotor(Constants.intakeWheelSpeed), intakeSubsystem))
      );

      NamedCommands.registerCommand(
        "Retract", 
        new InstantCommand(() -> intakeSubsystem.setIntakePosition("retract"), intakeSubsystem)
      );
    }

    if (Robot.shooterEnabled && Robot.intakeEnabled) {
      NamedCommands.registerCommand("ShootSpeaker", shooterAutonCommand);
    }
    
    // Configure the button bindings    
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
    if (Robot.swerveEnabled) {
      // OI.Driver.getOrientationButton().onTrue(new InstantCommand(swerveSubsystem::toggleOrientation));
      new JoystickButton(OI.Driver.kJoystick,8).onTrue(new InstantCommand(() -> System.out.println(swerveSubsystem.getHeading())));
      
      OI.Driver.getZeroButton().onTrue(new InstantCommand(swerveSubsystem::zeroHeading));
      OI.Driver.getAlignForwardButton().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(0), swerveSubsystem));
      OI.Driver.getAlignBackButton().onTrue(new InstantCommand(() -> swerveSubsystem.enableRotationHold(180), swerveSubsystem));
    }

    
    /* Testing */
    boolean spinAngleMotors = true; // used to determine direction of angle motors
    boolean spinDriveMotors = true; // used to determine direction of drive motors

    new JoystickButton(OI.Driver.kJoystick, 3).onTrue(
      new InstantCommand(() -> {
        swerveSubsystem.printModules();
      }));
    
    if (spinAngleMotors) {
      new JoystickButton(OI.Driver.kJoystick, 1).whileTrue(
        new StartEndCommand(() -> swerveSubsystem.spinAngleMotors(), () -> swerveSubsystem.stopModules(), swerveSubsystem)
      );
    }

    if (spinDriveMotors) {
      new JoystickButton(OI.Driver.kJoystick, 2).whileTrue(
        new StartEndCommand(() -> swerveSubsystem.spinDriveMotors(), () -> swerveSubsystem.stopModules(), swerveSubsystem)
      );
    }

    


  


    if (Robot.shooterEnabled) {
      OI.Driver.getShooterButton().whileTrue(shooterCommand);
    }

    if (Robot.intakeEnabled) {
      OI.Driver.getIntakeButton().whileTrue(new StartEndCommand(() -> intakeSubsystem.runIntakeMotor(Constants.intakeWheelSpeed), () -> intakeSubsystem.runIntakeMotor(0), intakeSubsystem));
      OI.Driver.getOuttakeButton().whileTrue(new StartEndCommand(() -> intakeSubsystem.runIntakeMotor(Constants.outtakeWheelSpeed), () -> intakeSubsystem.runIntakeMotor(0), intakeSubsystem));
      OI.Driver.getIntakeDeployButton().onTrue(new StartEndCommand(() -> intakeSubsystem.toggleIntakePosition(), () -> {}, intakeSubsystem));
      OI.Driver.getAmpShootButton().onTrue(new StartEndCommand(() -> intakeSubsystem.setIntakePosition("amp"), () -> {}, intakeSubsystem));
      OI.Driver.getIntakeSourceButton().onTrue(new StartEndCommand(() -> intakeSubsystem.setIntakePosition("source"), () -> {}, intakeSubsystem));
    }

    if (Robot.climbEnabled) {
      OI.Driver.getClimbToggleButton().onTrue(climbArmModeChooser.getSelected());

      OI.Driver.getClimbRetractButton().onTrue(new InstantCommand(
        () -> climbSubsystem.retractClimb(),
        climbSubsystem
      ));
    }

    OI.putControllerButtons();
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    if (Robot.swerveEnabled){
      return autonChooser.getSelected();
    }
    return null;
  }
}