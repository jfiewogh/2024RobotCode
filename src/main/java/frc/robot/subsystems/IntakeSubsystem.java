// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
    boolean deployed = false;
    
    private SparkMax intakeFeederMotor;
    private SparkMax intakeDeployMotor;
    private SparkClosedLoopController intakeDeployMotorPID;
    private RelativeEncoder intakeDeployMotorEncoder;

    public IntakeSubsystem() {
        intakeFeederMotor = MotorController.constructMotor(MotorConfig.IntakeMotorScooper);
        intakeDeployMotor = MotorController.constructMotor(MotorConfig.IntakeMotorRetract, Constants.intakeDeployPID);
        
        intakeDeployMotorPID = intakeDeployMotor.getClosedLoopController();
        intakeDeployMotorEncoder = intakeDeployMotor.getEncoder();
        intakeDeployMotorEncoder.setPosition(0);

        setIntakePosition("retract");  // Reset position to retract (retract is not 0)
        runIntakeMotor(0);                // & ensure intake motor is off at initializiation
    }

    public void runIntakeMotor(double speed) {
        intakeFeederMotor.set(speed);
    }

    public void setIntakePosition(String position) { // the map has a bunch of positions to pick from
        intakeDeployMotorPID.setReference(Constants.positionMap.get(position), SparkMax.ControlType.kPosition);
    }

    public void toggleIntakePosition() {
        if (deployed) {
            setIntakePosition("retract");
        } 
        else if (!deployed) {
           setIntakePosition("deploy");
        }
        deployed = !deployed;
    }

    public boolean getIntakeDeployStatus() {
        return deployed;
    }
}