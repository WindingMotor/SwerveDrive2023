package frc.robot.util;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;

public class LightStrip {
    
    private AddressableLED leftStrip;
    private AddressableLED rightStrip;

    private AddressableLEDBuffer stripBuffer;
    private int rainbowFirstPixelHue = 0;

    public LightStrip(){

        // Create LED strip object
        leftStrip = new AddressableLED(5);
        rightStrip = new AddressableLED(6);

        // Create LED buffer object
        stripBuffer = new AddressableLEDBuffer(60);

        // Set LED strip length to buffer
        leftStrip.setLength(stripBuffer.getLength());
        rightStrip.setLength(stripBuffer.getLength());

    }

    // Abstraction method for setting entire strip color
    private void setStripColor(int r, int g, int b){
        for(var i = 0; i < stripBuffer.getLength(); i++){
            stripBuffer.setRGB(i, r, g, b);
        }

        leftStrip.setData(stripBuffer);
        rightStrip.setData(stripBuffer);
    }

    // Set entire strip to rainbow
    public void updateStripRainbow(){
        // Set strip colors
        for(var i = 0; i < stripBuffer.getLength(); i++){
            // Caculate hue value
            final var hue = (rainbowFirstPixelHue + (i * 180 / stripBuffer.getLength())) % 180;
            // Set strip buffer
            stripBuffer.setHSV(i, hue, 255, 128);
        }
        // Move rainbow
        rainbowFirstPixelHue += 3;
        // Check bounds
        rainbowFirstPixelHue %= 180;

        // Set led strip to new buffer
        leftStrip.setData(stripBuffer);
        rightStrip.setData(stripBuffer);
    }

    public void setRed(){ setStripColor(255, 0, 0);}

    public void setGreen(){ setStripColor(0, 255, 0);}

    public void setBlue(){ setStripColor(0, 0, 255);}

    public void setYellow(){ setStripColor(255, 255, 0);}

    public void setPurple(){ setStripColor(255, 0, 255);}


}
