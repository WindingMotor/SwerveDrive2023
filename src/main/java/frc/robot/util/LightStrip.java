// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightStrip  extends SubsystemBase{
    
    private AddressableLED strip;
    //private AddressableLED rightStrip;

    private AddressableLEDBuffer stripBuffer;
    private int rainbowFirstPixelHue = 0;

    private int stripLen = 90;
    public int[] currentColor = {0,0,0};

    public LightStrip(Joystick tx16s, int port){

        strip = new AddressableLED(port);
        //rightStrip = new AddressableLED(0);

        // Create LED buffer object
        stripBuffer = new AddressableLEDBuffer(stripLen);

        // Set LED strip length to buffer
        strip.setLength(stripBuffer.getLength());
        //rightStrip.setLength(stripBuffer.getLength());
    
        strip.setData(stripBuffer);
        //rightStrip.setData(stripBuffer);

        setGreen();
        setStripColor();

        startStrips();

    }

    @Override
    public void periodic() {

        //setGreen();
        //setStripColor(currentColor[0],currentColor[1],currentColor[2]);

        
        if(!DriverStation.isEnabled()){
             // Play rainbow during disabled
            updateStripRainbow();
        }else{
            setStripColor(currentColor[0],currentColor[1],currentColor[2]);
        }
    }

     // Set entire strip to the color variable
    private void setStripColor(){
        for(var i = 0; i < stripBuffer.getLength(); i++){
            stripBuffer.setRGB(i, currentColor[0],currentColor[1],currentColor[2]);}
        strip.setData(stripBuffer);
        //rightStrip.setData(stripBuffer);
    }

     // Set entire strip to a selected color
    private void setStripColor(int r, int g, int b){
        for(var i = 0; i < stripBuffer.getLength(); i++){
            stripBuffer.setRGB(i, r, g, b);}
        strip.setData(stripBuffer);
       // rightStrip.setData(stripBuffer);
    }

    // Set enrite strip to one selected color up to a point
    private void setStripColor(int r, int g, int b, int stopPoint){

        // Filtering and limiting stopPoint
        stopPoint = Math.abs(stopPoint);
        if(stopPoint>=stripLen){stopPoint = stripLen;}
        if(stopPoint<=0){stopPoint = 0;}

        // Set led colors
        for(var i = 0; i < stripBuffer.getLength(); i++){
            if(i >= stopPoint){stripBuffer.setRGB(i, 0, 0, 0);}else{
                stripBuffer.setRGB(i, r, g, b);
            }
            }
        strip.setData(stripBuffer);
        //rightStrip.setData(stripBuffer);
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
        strip.setData(stripBuffer);
        //rightStrip.setData(stripBuffer);
        //rightStrip.setData(stripBuffer);
    }

    public void startStrips(){
        strip.start();
       // rightStrip.start();
    }

    public void stopStrips(){
        strip.stop();
        //rightStrip.start();
    }
    

    public void setRed(){ currentColor = new int[]{255,0,0};}

    public void setRGB(int r, int g, int b){ currentColor = new int[]{r,g,b};}

    public void setGreen(){ currentColor = new int[]{0,255,0};}

    public void setBlue(){ currentColor = new int[]{0,0,255};}

    // public void setYellow(){ setStripColor(255, 255, 0);}
    public void setYellow(){currentColor = new int[]{255,255,0};}

    //public void setPurple(){ setStripColor(255, 0, 255);}
    public void setPurple(){currentColor = new int[]{255,0,255};}

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
