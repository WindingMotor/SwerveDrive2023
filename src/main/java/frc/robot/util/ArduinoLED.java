
package frc.robot.util;
import edu.wpi.first.wpilibj.DigitalOutput;

public class ArduinoLED {

    private DigitalOutput red;
    private DigitalOutput green;
    private DigitalOutput blue;
    private DigitalOutput flash;
    private DigitalOutput special;

    public ArduinoLED(){
        
        red = new DigitalOutput(1);
        green = new DigitalOutput(2);
        blue = new DigitalOutput(3);
        flash = new DigitalOutput(4);
        special = new DigitalOutput(5);
        
    }

//-----------------------------------// Black & White

    public void setOff(){
        red.set(false);
        green.set(false);
        blue.set(false);
        flash.set(false);
        special.set(false);
    }

    public void setWhite(){
        red.set(true);
        green.set(true);
        blue.set(true);
        flash.set(false);
        special.set(false);
    }

    public void setWhiteFlash(){
        red.set(true);
        green.set(true);
        blue.set(true);
        flash.set(true);
        special.set(false);
    }

//-----------------------------------// Primary Colors

    public void setRed(){
        red.set(true);
        green.set(false);
        blue.set(false);
        flash.set(false);
        special.set(false);
    }

    public void setRedFlash(){
        red.set(true);
        green.set(false);
        blue.set(false);
        flash.set(true);
        special.set(false);
    }

    public void setGreen(){
        red.set(false);
        green.set(true);
        blue.set(false);
        flash.set(false);
        special.set(false);
    }

    public void setGreenFlash(){
        red.set(false);
        green.set(true);
        blue.set(false);
        flash.set(true);
        special.set(false);
    }
    
    public void setBlue(){
        red.set(false);
        green.set(false);
        blue.set(true);
        flash.set(false);
        special.set(false);
    }

    public void setBlueFlash(){
        red.set(false);
        green.set(false);
        blue.set(true);
        flash.set(true);
        special.set(false);
    }

//-----------------------------------// Secondary Colors

    public void setYellow(){
        red.set(true);
        green.set(true);
        blue.set(false);
        flash.set(false);
        special.set(false);
    }

    public void setYellowFlash(){
        red.set(true);
        green.set(true);
        blue.set(false);
        flash.set(true);
        special.set(false);
    }

    public void setPurple(){
        red.set(true);
        green.set(false);
        blue.set(true);
        flash.set(false);
        special.set(false);
    }

    public void setPurpleFlash(){
        red.set(true);
        green.set(false);
        blue.set(true);
        flash.set(true);
        special.set(false);
    }

    public void setTeal(){
        red.set(false);
        green.set(true);
        blue.set(true);
        flash.set(false);
        special.set(false);
    }

    public void setTealFlash(){
        red.set(false);
        green.set(true);
        blue.set(true);
        flash.set(true);
        special.set(false);
    }

//-----------------------------------// Speical

    public void setRainbow(){
        red.set(false);
        green.set(false);
        blue.set(false);
        flash.set(false);
        special.set(true);
    }

    public void setRedWhite(){
        red.set(false);
        green.set(false);
        blue.set(false);
        flash.set(true);
        special.set(true);
    }

    public void setBlueWhite(){
        red.set(false);
        green.set(false);
        blue.set(true);
        flash.set(true);
        special.set(true);
    }

}
