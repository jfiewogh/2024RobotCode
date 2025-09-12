// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc.robot.Constants;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;

public class ShooterSubsystem extends SubsystemBase {
    // RPM Variables
    private double targetRPM;
    private double currentRPM;
    private double smoothRPM;

    // Flywheel Controllers
    private SparkMax flywheelController;
    private SparkMax flywheelController2;
    private SimpleMotorFeedforward flywheelFF;
    private SparkClosedLoopController flywheelPID;
    private RelativeEncoder flywheelEncoder;

    public ShooterSubsystem() {
        smoothRPM = 0;
        
        flywheelController = MotorController.constructMotor(MotorConfig.LeftShooterMotor, Constants.shooterPIDArray);
        flywheelController2 = MotorController.constructMotor(MotorConfig.RightShooterMotor);

        flywheelFF = new SimpleMotorFeedforward(Constants.kSg, Constants.kVg, Constants.kAg);
        flywheelPID = flywheelController.getClosedLoopController();
        flywheelController2.configure(new SparkMaxConfig()
            .apply(new ClosedLoopConfig().outputRange(0, 1))
            .follow(flywheelController), null, null);

        flywheelEncoder = flywheelController.getEncoder();
    }

    public void spinFlywheel(double rpm) {
        if (rpm == 0) {
            flywheelPID.setReference(0, SparkMax.ControlType.kVoltage);
        } else {
            targetRPM = rpm;
            flywheelPID.setReference(
                rpm, 
                SparkMax.ControlType.kVelocity, 
                ClosedLoopSlot.kSlot0, 
                flywheelFF.calculate(rpm / 60.0)
            );
        }
    }

    public void primeFlywheel() {
        spinFlywheel(Constants.fixedShooterRPM);
    }
    
    public boolean flywheelReady() {
        return (smoothRPM > targetRPM - Constants.tolerance && smoothRPM < targetRPM + Constants.tolerance);
    }

    @Override
    public void periodic() {
        currentRPM = flywheelEncoder.getVelocity();
        smoothRPM = (currentRPM * Constants.smoothingAlpha) + (smoothRPM * (1 - Constants.smoothingAlpha));
    }
}