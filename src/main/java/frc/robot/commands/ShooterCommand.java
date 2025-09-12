// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.ShooterSubsystem;
import frc.robot.subsystems.IntakeSubsystem;
import frc.robot.subsystems.BlinkinLedSubsystem;
import frc.robot.subsystems.BlinkinLedSubsystem.BlinkinMode;
import frc.robot.Constants;

public class ShooterCommand extends Command {
    private IntakeSubsystem intakeSubsystem;
    private ShooterSubsystem shooterSubsystem;
    private BlinkinLedSubsystem ledSubsystem;
    private boolean ledEnabled;

    public ShooterCommand(IntakeSubsystem intakeSubsystem, ShooterSubsystem shooterSubsystem, BlinkinLedSubsystem ledSubsystem, boolean ledEnabled) {
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
}

    @Override
    public void execute() {
        shooterSubsystem.primeFlywheel();
        if (shooterSubsystem.flywheelReady()) {
            if (ledEnabled) {
                ledSubsystem.setMode(BlinkinMode.ShooterReady);
            }
            intakeSubsystem.runIntakeMotor(Constants.outtakeWheelSpeed);
        }
    }

    @Override
    public void end(boolean interrupted) {
        shooterSubsystem.spinFlywheel(0);
        if (ledEnabled) {
            ledSubsystem.setMode(BlinkinMode.Off);
        }
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}