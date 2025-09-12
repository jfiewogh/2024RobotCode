package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkClosedLoopController;
import frc.robot.hardware.MotorController;
import frc.robot.hardware.MotorController.MotorConfig;
import frc.robot.Constants.ClimbPosition;

public class ClimbSubsystem extends SubsystemBase {
    private SparkMax leftArmMotor;
    private SparkMax rightArmMotor;

    private SparkClosedLoopController leftArmMotorPID;
    private SparkClosedLoopController rightArmMotorPID;

    private boolean isDeployed = false;

    public ClimbSubsystem() {
        leftArmMotor = MotorController.constructMotor(MotorConfig.LeftArm);
        rightArmMotor = MotorController.constructMotor(MotorConfig.RightArm);

        leftArmMotorPID = leftArmMotor.getClosedLoopController();
        rightArmMotorPID = rightArmMotor.getClosedLoopController();

        retractClimb();
    }

    public void extendClimb(ClimbPosition climbPosition) {
        leftArmMotorPID.setReference(climbPosition.getleftArmPosition(), SparkMax.ControlType.kPosition);
        rightArmMotorPID.setReference(climbPosition.getRightArmPosition(), SparkMax.ControlType.kPosition);

        isDeployed = true;
    }

    public void retractClimb() {
        leftArmMotorPID.setReference(0, SparkMax.ControlType.kPosition);
        rightArmMotorPID.setReference(0, SparkMax.ControlType.kPosition);

        isDeployed = false;
    }

    public void retractClimb(ClimbPosition climbPosition) {
        leftArmMotorPID.setReference(climbPosition.getLeftArmDownPosition(), SparkMax.ControlType.kPosition);
        rightArmMotorPID.setReference(climbPosition.getRightArmDownPosition(), SparkMax.ControlType.kPosition);

        isDeployed = false;
    }

    public void toggleClimb(ClimbPosition climbPosition) {
        if (isDeployed) {
            retractClimb(climbPosition);
        } else {
            extendClimb(climbPosition);
        }
    }
}
