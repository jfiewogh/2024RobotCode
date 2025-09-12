// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants;
import frc.robot.subsystems.BlinkinLedSubsystem;
import frc.robot.subsystems.BlinkinLedSubsystem.BlinkinMode;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.ShooterSubsystem;

public class ShooterAutonCommand extends Command {
    private final int maxPrimeCycles = 100; // max number of 20ms cycles to wait before forcing command exit
    private int currentCycleCount = 0;

    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private BlinkinLedSubsystem ledSubsystem;
    private boolean ledEnabled;

    public ShooterAutonCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, BlinkinLedSubsystem ledSubsystem, boolean ledEnabled) {
        this.intakeSubsystem = intakeSubsystem;
        this.shooterSubsystem = shooterSubsystem;
        this.ledSubsystem = ledSubsystem;
        this.ledEnabled = ledEnabled;

        addRequirements(shooterSubsystem);
        addRequirements(intakeSubsystem);
        if (ledEnabled) {
            addRequirements(ledSubsystem);
        }
    }
  
    @Override
    public void initialize() {
        shooterSubsystem.primeFlywheel(); 
    }

    @Override
    public void execute() {
        currentCycleCount++;
    }

    @Override
    public void end(boolean interrupted) {
        if (ledEnabled) {
            ledSubsystem.setMode(BlinkinMode.ShooterReady);
        }
        intakeSubsystem.runIntakeMotor(Constants.outtakeWheelSpeed);
        shooterSubsystem.spinFlywheel(0);
    }
  
    @Override
    public boolean isFinished() {
        return shooterSubsystem.flywheelReady() || currentCycleCount >= maxPrimeCycles;
    }
}