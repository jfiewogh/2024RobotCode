package frc.robot.subsystems;

import edu.wpi.first.networktables.GenericEntry;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;

import java.util.Map;

public class LedSubsystem {
    private AddressableLED led;
    private AddressableLEDBuffer ledBuffer;

    private static final int port = 2;
    private static final int length = 256;

    // colors
    public final Color redColor = new Color(255, 0, 0);
    public final Color blueColor = new Color(0, 0, 255);
    public final Color offColor = new Color(0, 0, 0);

    // tabs
    private ShuffleboardTab ledTab = Shuffleboard.getTab("Led");
    private GenericEntry ledColor = ledTab.add("Color", "Off").getEntry();
    private GenericEntry ledBrightness = ledTab.add("Brightness", 0)
        .withWidget(BuiltInWidgets.kNumberSlider)
        .withProperties(Map.of("min", 0, "max", 1))
        .getEntry();
    
    public static enum Mode{
        RED,
        BLUE,
        RAINBOW,
        OFF
    }

    public LedSubsystem(){
        led = new AddressableLED(port);
        ledBuffer = new AddressableLEDBuffer(length);

        led.setLength(length);

        led.setData(ledBuffer);
        led.start();
    }

    public void setMode(Mode mode){
        switch(mode){
            case RED:
                setSolidColor(redColor);
                ledColor.setString("Red");
                break;
            case BLUE:
                setSolidColor(blueColor);
                ledColor.setString("Blue");
                break;
            case RAINBOW:
                setRainbowColor();
                ledColor.setString("Rainbow");
                break;
            case OFF:
                setSolidColor(offColor);
                ledColor.setString("Off");
                break;
        }
        led.setData(ledBuffer);
    }

    private void setSolidColor(Color color){
        double brightness = ledBrightness.getDouble(0.5);
        double r = color.red * brightness;
        double g = color.green * brightness;
        double b = color.blue * brightness;
        // set all the leds to color
        for(int i = 0; i < length; i++){
            ledBuffer.setRGB(i, (int)r, (int)g, (int)b);
        }
    }

    private void setRainbowColor(){
        double brightness = ledBrightness.getDouble(0.5);
        for(int i = 0; i < length; i++){
            double hue = (i/length) * 180;
            double value = brightness * 255;
            ledBuffer.setHSV(i, (int)hue, 255, (int)value);
        }
    }
}
