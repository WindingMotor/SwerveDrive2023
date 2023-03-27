
package frc.robot.util;
import java.sql.Driver;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightStrip  extends SubsystemBase{
    
    private AddressableLED leftStrip;
   // private AddressableLED rightStrip;

    private AddressableLEDBuffer stripBuffer;
    private int rainbowFirstPixelHue = 0;
    private Joystick tx16s;

    public LightStrip(Joystick tx16s){
        this.tx16s = tx16s;
        // Create LED strip object
        leftStrip = new AddressableLED(0);
       // rightStrip = new AddressableLED(portTwo);

        // Create LED buffer object
        stripBuffer = new AddressableLEDBuffer(45);

        // Set LED strip length to buffer
        leftStrip.setLength(stripBuffer.getLength());
       // rightStrip.setLength(stripBuffer.getLength());
    
        stripBuffer.setRGB(2, 255, 0, 0);
        stripBuffer.setRGB(3, 0, 255, 0);
        stripBuffer.setRGB(4, 0, 0, 255);

        leftStrip.setData(stripBuffer);
        setGreen();
        startStrips();

    }

    @Override
    public void periodic() {
        // This method will be called once per scheduler run
        //updateYellowPurple();
        //updateStripRainbow();

        if(!DriverStation.isEnabled()){
            updateStripRainbow();
        }
   

        /* 
        if(DriverStation.isEnabled()){
            setGreen();
        }else{
            setRed();
        }*/
        
      }
    
     // Abstraction method for setting entire strip color
    private void setStripColor(int r, int g, int b){
        for(var i = 0; i < stripBuffer.getLength(); i++){
            stripBuffer.setRGB(i, r, g, b);
        }

        leftStrip.setData(stripBuffer);
      //  rightStrip.setData(stripBuffer);
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
        //rightStrip.setData(stripBuffer);
    }

    public void startStrips(){
        leftStrip.start();
    }

    public void stopStrips(){
        leftStrip.stop();
    }
    

    public void setRed(){ setStripColor(255, 0, 0);}

    public void setGreen(){ setStripColor(0, 255, 0);}

    public void setBlue(){ setStripColor(0, 0, 255);}

    public void setYellow(){ setStripColor(255, 255, 0);}

    public void setPurple(){ setStripColor(255, 0, 255);}

    public void updateYellowPurple(){
        setYellow();
        Timer.delay(2);
        setPurple();
        Timer.delay(2);
    }

    public void updateRedBlue(){
        setRed();
        Timer.delay(2);
        setBlue();
        Timer.delay(2);
    }

    public void updateWhiteBlack(){
        setStripColor(255, 255, 255);
        Timer.delay(0.2);
        setStripColor(0, 0, 0);
        Timer.delay(0.2);
    }
}
