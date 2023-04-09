
package frc.robot.util;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class LightStrip  extends SubsystemBase{
    
    private AddressableLED leftStrip;
   // private AddressableLED rightStrip;

    private AddressableLEDBuffer stripBuffer;
    private int rainbowFirstPixelHue = 0;

    private int stripLen = 45;
    public int[] currentColor = {0,0,0};

    private PowerDistribution PDP = new PowerDistribution(1, ModuleType.kRev);
    //private Joystick tx16s;

    public LightStrip(Joystick tx16s){

        leftStrip = new AddressableLED(0);
       // rightStrip = new AddressableLED(portTwo);

        // Create LED buffer object
        stripBuffer = new AddressableLEDBuffer(stripLen);

        // Set LED strip length to buffer
        leftStrip.setLength(stripBuffer.getLength());
       // rightStrip.setLength(stripBuffer.getLength());
    
        leftStrip.setData(stripBuffer);

        setGreen();
        setStripColor();

        startStrips();

    }

    @Override
    public void periodic() {

        if(!DriverStation.isEnabled()){
             // Play rainbow during disabled
            updateStripRainbow();
        }else{
<<<<<<< HEAD
            setStripColor(currentColor[0],currentColor[1],currentColor[2]);
        }

        /* 
        if(DriverStation.isEnabled()){
            setGreen();
        }else{
            setRed();
        }*/
        
=======
        if(PDP.getVoltage() < 10){
            // Modify color if voltage is dropping very low
            setStripColor(currentColor[0] + (int) PDP.getVoltage() * 2,currentColor[1],currentColor[2],(int) PDP.getTotalCurrent() / 2);
        } else{ setStripColor(currentColor[0],currentColor[1],currentColor[2],(int) PDP.getTotalCurrent() / 2); } // Normal operation for signaling
        }
>>>>>>> 66bfe31c1680bbaa2a2e66f5f9c859edb585227d
      }

    
     // Set entire strip to the color variable
    private void setStripColor(){
        for(var i = 0; i < stripBuffer.getLength(); i++){
            stripBuffer.setRGB(i, currentColor[0],currentColor[1],currentColor[2]);}
        leftStrip.setData(stripBuffer);
    }

     // Set entire strip to a selected color
    private void setStripColor(int r, int g, int b){
        for(var i = 0; i < stripBuffer.getLength(); i++){
            stripBuffer.setRGB(i, r, g, b);}
        leftStrip.setData(stripBuffer);
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
        leftStrip.setData(stripBuffer);
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
    

    public void setRed(){ currentColor = new int[]{255,0,0};}

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
