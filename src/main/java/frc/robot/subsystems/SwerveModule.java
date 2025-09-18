// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.hardware.CANcoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.system.plant.LinearSystemId;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.datalog.DataLog;
import edu.wpi.first.util.datalog.DoubleLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Robot;
import frc.robot.classes.RelativeEncoderSim;
import frc.robot.hardware.AbsoluteEncoder;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import frc.robot.hardware.AbsoluteEncoder.EncoderConfig;

public class SwerveModule extends SubsystemBase {
    public static final double kMaxSpeedMetersPerSecond = Units.feetToMeters(14.5); // Motor speed when set to 1.0
    public static final double kWheelDiameterMeters = Units.inchesToMeters(3.5);
    public static final double kDriveMotorGearRatio = (14.0 / 50.0) * (27.0 / 17.0) * (15.0 / 45.0);
    public static final double kTurningMotorGearRatio = (14.0 / 50.0) * (10.0 / 60.0);
    public static final double kDriveEncoderRotFactor = kDriveMotorGearRatio * Math.PI * kWheelDiameterMeters; //Conversion factor converting the Drive Encoder's rotations to meters
    public static final double kDriveEncoderRPMFactor = kDriveEncoderRotFactor / 60; //Conversion factor converting the Drive Encoder's RPM to meters per second
    public static final double kTurningEncoderRotFactor = kTurningMotorGearRatio * 2 * Math.PI; //Conversion factor converting the Turn Encoder's rotations to Radians
    public static final double kTurningEncoderRPMFactor = kTurningEncoderRotFactor / 60; //Conersion factor converting the Turn Encoder's RPM to radians per second
    public static final double kPTurning = 0.4; //P gain for the turning motor

    private final SparkMax driveMotor;
    private final SparkMax turningMotor;

    private final RelativeEncoder driveEncoder;
    private final RelativeEncoder turningEncoder;

    private final FlywheelSim simTurningMotor;
    private final FlywheelSim simDriveMotor;

    private final RelativeEncoderSim simDriveEncoder;
    private final RelativeEncoderSim simTurningEncoder;

    private final SparkClosedLoopController turningPIDController;
    private final PIDController simTurningPIDController;
    private double turningSetpoint;

    private final CANcoder absoluteEncoder;
    private final EncoderConfig absoluteEncoderConfig;

    private DataLog datalog = DataLogManager.getLog();
    private DoubleLogEntry desiredSpeedLog;
    private DoubleLogEntry actualSpeedLog;
    private DoubleLogEntry desiredAngleLog;
    private DoubleLogEntry actualAngleLog;
    private DoubleLogEntry rotationSpeedLog;

    private final String ID;

    public SwerveModule(MotorConfig driveMotorConfig, MotorConfig turningMotorConfig, EncoderConfig absoluteEncoderConfig, String ID) {

        this.ID = ID;

        absoluteEncoder = Robot.isSimulation() ? null : AbsoluteEncoder.constructCANCoder(absoluteEncoderConfig);
        this.absoluteEncoderConfig = absoluteEncoderConfig;

        driveMotor = MotorController.constructMotor(driveMotorConfig);
        turningMotor = MotorController.constructMotor(turningMotorConfig);

        driveEncoder = this.driveMotor.getEncoder();
        turningEncoder = this.turningMotor.getEncoder();

        com.revrobotics.spark.config.EncoderConfig driveEncoderConfig = new com.revrobotics.spark.config.EncoderConfig()
            .positionConversionFactor(kDriveEncoderRotFactor)
            .velocityConversionFactor(kDriveEncoderRPMFactor);
        driveMotor.configure(new SparkMaxConfig().apply(driveEncoderConfig), null, null);

        com.revrobotics.spark.config.EncoderConfig turningEncoderConfig = new com.revrobotics.spark.config.EncoderConfig()
            .positionConversionFactor(kTurningEncoderRotFactor)
            .positionConversionFactor(kTurningEncoderRPMFactor);
        turningMotor.configure(new SparkMaxConfig().apply(turningEncoderConfig), null, null);

        simDriveMotor = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(2, 1.24), //TODO: Update with real SysID
            DCMotor.getNEO(1),
            kDriveMotorGearRatio
        );
        simTurningMotor = new FlywheelSim(
            LinearSystemId.identifyVelocitySystem(0.16, 0.0348), //TODO: Update with real SysID
            DCMotor.getNEO(1),
            kTurningMotorGearRatio
        );

        simDriveEncoder = new RelativeEncoderSim(driveMotor);
        simTurningEncoder = new RelativeEncoderSim(turningMotor);

        turningPIDController = turningMotor.getClosedLoopController();
        simTurningPIDController = new PIDController(2, 0, 0);
            // turningPIDController.getP(), turningPIDController.getI(), turningPIDController.getD());

        simTurningPIDController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();

        desiredSpeedLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setSpeed"); //Logs desired speed in meters per second
        actualSpeedLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setSpeed"); //Logs actual speed in meters per second
        desiredAngleLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/setAngle"); //Logs desired angle in radians
        actualAngleLog = new DoubleLogEntry(datalog, "/swerve/" + ID +"/actualAngle"); //Logs actual relative angle in radians
    }

    public String getID() {
        return ID;
    }

    public SwerveModulePosition getPosition() {
        return new SwerveModulePosition(getDrivePosition(), Rotation2d.fromRadians(getTurningPosition()));
    }

    public double getDrivePosition() {
        return Robot.isSimulation() ? simDriveEncoder.getPosition() : driveEncoder.getPosition();
    }

    public double getTurningPosition() {
        //The math for this remainder is position - (2pi * Math.round(position/2pi))
        return Robot.isSimulation() ? Math.IEEEremainder(simTurningEncoder.getPosition(), Math.PI * 2) : Math.IEEEremainder(turningEncoder.getPosition(), Math.PI * 2);
    }

    public double getAbsoluteTurningPosition() { 
        return Units.rotationsToRadians(Robot.isSimulation() ? 0 : absoluteEncoder.getAbsolutePosition().getValueAsDouble());
    }

    public double getDriveVelocity() {
        return Robot.isSimulation() ? simDriveEncoder.getVelocity() : driveEncoder.getVelocity();
    }
    
    public double getTurningVelocity() {
        double currentRotation = Robot.isSimulation() ? simTurningEncoder.getVelocity() : turningEncoder.getVelocity();
        rotationSpeedLog.append(currentRotation);
        return currentRotation;
    }

    public void resetEncoders() {
        driveEncoder.setPosition(0);
        turningEncoder.setPosition(getAbsoluteTurningPosition());
        if(Robot.isReal()){
            absoluteEncoder.close();
        }
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    private double calculateSetpoint(double stateAngle){
        return turningEncoder.getPosition() + Math.IEEEremainder(stateAngle - getTurningPosition(), 2 * Math.PI); 
    }

    public void setDesiredState(SwerveModuleState desiredState) {
        if (Math.abs(desiredState.speedMetersPerSecond) == 0) { //Prevent wheels to returning to heading of 0 when controls released
            stop();
            return;
        }

        desiredState.optimize(Rotation2d.fromRotations(
            absoluteEncoder.getAbsolutePosition().getValueAsDouble()));

        double driveSpeed = desiredState.speedMetersPerSecond / kMaxSpeedMetersPerSecond;

        turningMotor.set(
            simTurningPIDController.calculate(
                absoluteEncoder.getAbsolutePosition().getValueAsDouble(), 
                desiredState.angle.getRotations()));

        driveMotor.set(driveSpeed);

        
        // turningPIDController.setReference(turningSetpoint, ControlType.kPosition);

        SmartDashboard.putString("Swerve[" + ID + "] state", desiredState.toString());
        desiredAngleLog.append(desiredState.angle.getRadians());
        desiredSpeedLog.append(desiredState.speedMetersPerSecond);
    }

    public void park(boolean reversed) {
        stop();
        if(reversed){
            //not using calculateSetpoint because these are less than one full rotation
            turningPIDController.setReference(Math.PI/4, ControlType.kPosition);  
        } else{
            turningPIDController.setReference(-Math.PI/4, ControlType.kPosition);
        }
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }

    public void coast() {
        driveMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
        turningMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kCoast), null, null);
    }

    public void brake() {
        driveMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
        turningMotor.configure(new SparkMaxConfig().idleMode(IdleMode.kBrake), null, null);
    }

    public String getStatusAbsoulteEncoder() {
        double offset = Robot.isCompetitionRobot 
            ? absoluteEncoderConfig.getCompetitionOffset()
            : absoluteEncoderConfig.getPracticeOffset();
        return String.format("%s: %.4f(rad) [off: %.4f] ", ID, getAbsoluteTurningPosition(), offset);
    }

    @Override
    public void simulationPeriodic() {
        simDriveMotor.setInputVoltage(driveMotor.get() * RobotController.getBatteryVoltage());
        simTurningMotor.setInputVoltage(turningMotor.get()  * RobotController.getBatteryVoltage());

        simDriveMotor.update(Robot.kDefaultPeriod);
        simTurningMotor.update(Robot.kDefaultPeriod);

        simDriveEncoder.setPosition(simDriveEncoder.getPosition() + simDriveMotor.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod);
        simDriveEncoder.setSimVelocity(simDriveMotor.getAngularVelocityRadPerSec());

        simTurningEncoder.setPosition(simTurningEncoder.getPosition() + simTurningMotor.getAngularVelocityRadPerSec() * Robot.kDefaultPeriod);
        simTurningEncoder.setSimVelocity(simTurningMotor.getAngularVelocityRadPerSec());

        turningMotor.set(simTurningPIDController.calculate(getTurningPosition(), turningSetpoint));
    }

    @Override
    public void periodic(){
        actualSpeedLog.append(driveEncoder.getVelocity());
        actualAngleLog.append(getTurningPosition());
    }

    /* Testing */

    public void setDriveSpeed(double speed) {
        driveMotor.set(speed);
    }

    public void setAngleSpeed(double speed) {
        turningMotor.set(speed);
    }
}
