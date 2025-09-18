// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import java.util.function.Supplier;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructPublisher;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.SwerveSubsystem;

public class SwerveTeleopCommand extends Command {

    private final SwerveSubsystem swerveSubsystem;
    private final Supplier<Double> xJoystick, yJoystick, rJoystick;

    StructPublisher<ChassisSpeeds> speedsPublisher = NetworkTableInstance.getDefault()
        .getStructTopic("SwerveTeleopCommand/ChassisSpeeds", ChassisSpeeds.struct).publish();

    public SwerveTeleopCommand(SwerveSubsystem swerveSubsystem,
            Supplier<Double> xJoystick, Supplier<Double> yJoystick, Supplier<Double> rJoystick) {
        this.swerveSubsystem = swerveSubsystem;
        this.xJoystick = xJoystick;
        this.yJoystick = yJoystick;
        this.rJoystick = rJoystick;
        addRequirements(swerveSubsystem);
    }

    @Override
    public void initialize() {

    }

    @Override
    public void execute() {
        SwerveModuleState[] moduleStates = 
            swerveSubsystem.convertToModuleStates(xJoystick.get(), yJoystick.get(), rJoystick.get());
        swerveSubsystem.setModuleStates(moduleStates);

        speedsPublisher.set(swerveSubsystem.getChassisSpeeds());
    }

    @Override
    public void end(boolean interrupted) {
        swerveSubsystem.stopModules();
    }

    @Override
    public boolean isFinished() {
        return false;
    }
}
