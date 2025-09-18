package frc.robot.hardware;

import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;

import frc.robot.Constants;

public class MotorController {
    public static final class MotorDefaults{
        //Constants to use as default values for Motor Controllers
        public static final int kCurrentLimit = 40;
        public static final double kOpenLoopRampRate = 0.2;
    }

    public static enum MotorConfig {

        // Swerve Modules
        FrontLeftModuleDrive(Constants.canIdFrontLeftMotorDrive, 50, IdleMode.kBrake, true),
        FrontLeftModuleTurn(Constants.canIdFrontLeftMotorPivot, 40, IdleMode.kBrake, true),
        FrontRightModuleDrive(Constants.canIdFrontRightMotorDrive, 50, IdleMode.kBrake, true),
        FrontRightModuleTurn(Constants.canIdFrontRightMotorPivot, 40, IdleMode.kBrake, true),
        BackLeftModuleDrive(Constants.canIdBackLeftMotorDrive, 50, IdleMode.kBrake),
        BackLeftModuleTurn(Constants.canIdBackLeftMotorPivot, 40, IdleMode.kBrake, true),
        BackRightModuleDrive(Constants.canIdBackRightMotorDrive, 50, IdleMode.kBrake),
        BackRightModuleTurn(Constants.canIdBackRightMotorPivot, 40, IdleMode.kBrake, true),

        // Climb Motors
        LeftArm(Constants.canIdClimberMotorLeft, 50, IdleMode.kBrake),
        RightArm(Constants.canIdClimberMotorRight, 50, IdleMode.kBrake),
        
        // Intake Motors
        IntakeMotorRetract(Constants.canIdIntakeMotorRetract, 30, IdleMode.kBrake),
        IntakeMotorScooper(Constants.canIdIntakeMotorScooper, 30, IdleMode.kBrake),

        // Shooter Motors (Placeholder values)
        LeftShooterMotor(Constants.canIdShooterMotorLeft, 40, IdleMode.kBrake),
        RightShooterMotor(Constants.canIdShooterMotorRight, 40, IdleMode.kBrake, true);

        private int ID;
        private int currentLimit;
        private IdleMode idleMode;
        private double openLoopRampRate;
        private boolean reversed;

        MotorConfig(int ID, int currentLimit, IdleMode idleMode, double openLoopRampRate, boolean reversed){
            this.ID = ID;
            this.currentLimit = currentLimit;
            this.idleMode = idleMode;
            this.openLoopRampRate = openLoopRampRate;
            this.reversed = reversed;
        }

        MotorConfig(int ID, int currentLimit, IdleMode idleMode, double openLoopRampRate){
            this(ID, currentLimit, idleMode, openLoopRampRate, false);
        }

        MotorConfig(int ID, int currentLimit, IdleMode idleMode, boolean reversed){
            this(ID, currentLimit, idleMode, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID, int currentLimit, IdleMode idleMode){
            this(ID, currentLimit, idleMode, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorConfig(int ID, int currentLimit, boolean reversed){
            this(ID, currentLimit, IdleMode.kCoast, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID, int currentLimit){
            this(ID, currentLimit, IdleMode.kCoast, MotorDefaults.kOpenLoopRampRate, false);
        }

        MotorConfig(int ID, boolean reversed){
            this(ID, MotorDefaults.kCurrentLimit, IdleMode.kCoast, MotorDefaults.kOpenLoopRampRate, reversed);
        }

        MotorConfig(int ID){
            this(ID, MotorDefaults.kCurrentLimit, IdleMode.kCoast, MotorDefaults.kOpenLoopRampRate, false);
        }

        public int getID(){
            return ID;
        }

        public int getCurrentLimit(){
            return currentLimit;
        }

        public IdleMode getIdleMode(){
            return idleMode;
        }

        public double getOpenLoopRampRate(){
            return openLoopRampRate;
        }

        public boolean getReversed(){
            return reversed;
        }
    }

    public static SparkMax constructMotor(MotorConfig config){
        SparkMax motor = new SparkMax(config.getID(), MotorType.kBrushless);
        SparkMaxConfig motorConfig = new SparkMaxConfig();
        motorConfig.smartCurrentLimit(config.getCurrentLimit());
        motorConfig.idleMode(config.getIdleMode());
        motorConfig.openLoopRampRate(config.getOpenLoopRampRate());
        motorConfig.inverted(config.getReversed());
        // motor.restoreFactoryDefaults();
        motor.configure(motorConfig, null, null);
        return motor;
    }

    public static SparkMax constructMotor(MotorConfig config, double[] PIDArray){
        SparkMax motor = constructMotor(config);
        motor.configure(new SparkMaxConfig().apply(new ClosedLoopConfig()
            .p(PIDArray[0]).i(PIDArray[1]).d(PIDArray[2])), 
            null, null);
        return motor;
    }
}
