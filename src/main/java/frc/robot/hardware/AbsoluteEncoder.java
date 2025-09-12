package frc.robot.hardware;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.configs.MagnetSensorConfigs;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.DutyCycle;
import frc.robot.Constants;
import frc.robot.Robot;

public class AbsoluteEncoder {
    public enum EncoderConfig {
        //Swerve Modules (CAN)
        //Offsets determined by manually turning all modules to 0 (forward) and recording their positions
        FrontLeftModule(Constants.canIdFrontLeftCancoder, false, -1.7441, -1.1520),
        FrontRightModule(Constants.canIdFrontRightCancoder, false, 2.0678, 2.0816 - (Math.PI)),
        BackLeftModule(Constants.canIdBackLeftCancoder, false, -2.0801 + (Math.PI), -0.9664),
        BackRightModule(Constants.canIdBackRightCancoder, false, 2.8041, -0.5906);

        private int ID;
        private boolean reversed;
        private double competitionOffset; //Offset in radians, used on competition bot
        private double practiceOffset; //Offset in radians, used on practice bot
    
        EncoderConfig(int ID, boolean reversed, double offset, double altOffset){
            this.ID = ID;
            this.reversed = reversed;
            this.competitionOffset = offset;
            this.practiceOffset = altOffset;
        }

        EncoderConfig(int ID, boolean reversed, double offset){
            this(ID, reversed, offset, 0);
        }
    
        EncoderConfig(int ID, boolean reversed){
            this(ID, reversed, 0, 0);
        }
    
        EncoderConfig(int ID){
            this(ID, false, 0, 0);
        }
    
        public int getID(){
            return ID;
        }

        public boolean getReversed(){
            return reversed;
        }

        public double getCompetitionOffset(){
            return competitionOffset;
        }

        public double getPracticeOffset(){
            return practiceOffset;
        }
    }

    public static CANcoder constructCANCoder(EncoderConfig config) {
        CANcoder encoder = new CANcoder(config.getID());
        CANcoderConfiguration canConfig = new CANcoderConfiguration();
        MagnetSensorConfigs magConfig = new MagnetSensorConfigs();

        magConfig.withAbsoluteSensorDiscontinuityPoint(0.5); // Signed_PlusMinusHalf
        if (config.getReversed()) 
            magConfig.withSensorDirection(SensorDirectionValue.Clockwise_Positive);
        else
            magConfig.withSensorDirection(SensorDirectionValue.CounterClockwise_Positive);

        magConfig.withMagnetOffset(Units.radiansToRotations(Robot.isCompetitionRobot ? config.getCompetitionOffset() : config.getPracticeOffset()));

        canConfig.withMagnetSensor(magConfig);
        encoder.getConfigurator().apply(canConfig);

        return encoder;
    }

    public static DutyCycleEncoder constructREVEncoder(EncoderConfig config) {
        DutyCycleEncoder encoder = new DutyCycleEncoder(config.getID());
        double offset = Units.radiansToRotations(Robot.isCompetitionRobot ? config.getCompetitionOffset() : config.getPracticeOffset());
        // Enforce rotations to range: [0.0 , 1.0)
        offset = Math.abs(offset - (int) offset);
        encoder.setDutyCycleRange(0.0, 1.0);
        encoder.setInverted(config.getReversed());
        return encoder;
    }

    public static double getPositionRadians(DutyCycleEncoder encoder){
        //Get position of a REV encoder in radians
        double position = encoder.get(); // + encoderOffset;
        // position = encoder.setInverted(config.getResev) < 0 ? 1 - position : position;
        // if(position < 0){
        //     position++;
        // }
        return Units.rotationsToRadians(position);
    }

    public static double getPositionRadians(DutyCycleEncoder encoder, int places){
        //Truncates measure to places decimal points
        return Math.round(getPositionRadians(encoder) * Math.pow(10, places)) / Math.pow(10, places);
    }
}
