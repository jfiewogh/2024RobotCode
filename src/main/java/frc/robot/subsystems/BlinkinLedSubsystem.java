package frc.robot.subsystems;

import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.networktables.GenericEntry;

public class BlinkinLedSubsystem extends SubsystemBase {
    private ShuffleboardTab ledTab = Shuffleboard.getTab("LED");
    private GenericEntry ledColor = ledTab.add("Color", 100.0).getEntry();

    private final Spark spark;

    public static enum BlinkinMode {
        HoldingPiece(0.65), // orange
        ShooterReady(0.77), // green
        Source(0.93), // white
        Amp(0.83), // blue
        Default(0.67), // yellow
        Off(0.99);
        
        private final double value;
        BlinkinMode(double value) {
            this.value = value;
        }
    }
    
    public BlinkinLedSubsystem() {
        spark = new Spark(0);
    }

    public void blinkinStopLed(){
        spark.set(BlinkinMode.Off.value);
    }

    public void setMode(BlinkinMode mode){
        ledColor.setDouble(mode.value);
        spark.set(mode.value);
    }
}