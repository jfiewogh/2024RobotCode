// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.util.Map;
import static java.util.Map.entry;

public final class Constants {


    // Hardware Constants
    public static final int canIdFrontLeftMotorDrive = 1;
    public static final int canIdFrontLeftMotorPivot = 2;
    public static final int canIdFrontRightMotorDrive = 3;
    public static final int canIdFrontRightMotorPivot = 4;
    public static final int canIdBackLeftMotorDrive = 5;
    public static final int canIdBackLeftMotorPivot = 6;
    public static final int canIdBackRightMotorDrive = 7;
    public static final int canIdBackRightMotorPivot = 8;
    public static final int canIdIntakeMotorRetract = 9;
    public static final int canIdIntakeMotorScooper = 10;
    public static final int canIdShooterMotorLeft = 11;
    public static final int canIdShooterMotorRight = 13;
    public static final int canIdClimberMotorLeft = 14;
    public static final int canIdClimberMotorRight = 15;

    public static final int canIdPowerDistributionUnit = 20;

    public static final int canIdPnumaticHub = 21;
    public static final int canIdLaserCan = 22;
    public static final int canIdFrontLeftCancoder = 23;
    public static final int canIdFrontRightCancoder = 24;
    public static final int canIdBackLeftCancoder = 25;
    public static final int canIdBackRightCancoder = 26;

    // Driving Constants
    public static final double driveSpeedScale = 0.32; // Use this to scale the driving speed

    // Shooter constants
    // Robot values
    public static final double shooterHeightInches = 23; // height of tip of shooter
    public static final double shooterWheelRadiusInches = 2;

    // Field values
    public static final double speakerHeightInches = 80.5;
    public static final double gravitationalConstantFeet = 32;

    // PID values
    public static final double shooterPIDArray[] = {1.4e-09, 3.0E-8, 0};

    // FF gains
    public static final double kSg = 0.56;
    public static final double kVg = 0.14;
    public static final double kAg = 0.0087876;

    // RPM Values
    public static final int fixedShooterRPM = 2000;
    public static final double smoothingAlpha = 0.35;
    public static final int tolerance = 75; // Allowed RPM deviation from setpoint
    public static final double power = 0; // changes arc to hit speaker at more direct angle

    // Intake Constants
    public static final double[] intakeDeployPID = {0.25, 0, 1};
    public static final Map<String, Integer> positionMap = Map.ofEntries(
        entry("retract", 5),
        entry("deploy", 70),
        entry("amp", 15),
        entry("source", 10)
    );

    public enum ClimbPosition {
        Down(0, 0, 0, 0),
        Uniform(100, 100, 75, 75),
        LeftElevated(100, 75, 75, 50),
        RightElevated(75, 100, 50, 75);

        private int leftArmPosition;
        private int rightArmPosition;
        private int leftArmDownPosition;
        private int rightArmDownPosition;

        ClimbPosition(int lArmPos, int rArmPos, int lArmDownPos, int rArmDownPos) {
            this.leftArmPosition = lArmPos;
            this.rightArmPosition = rArmPos;
            this.leftArmDownPosition = lArmDownPos;
            this.rightArmDownPosition = rArmDownPos;
        }

        public int getleftArmPosition() {
            return leftArmPosition;
        }

        public int getRightArmPosition() {
            return rightArmPosition;
        }

        public int getLeftArmDownPosition() {
            return leftArmDownPosition;
        }

        public int getRightArmDownPosition() {
            return rightArmDownPosition;
        }
    } 

    public static final double intakeWheelSpeed = 0.7;
    public static final double outtakeWheelSpeed = -0.7;
}