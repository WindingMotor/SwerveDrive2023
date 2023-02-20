
package frc.robot.util;
import edu.wpi.first.wpilibj.DigitalOutput;

public class ArduinoLED {

    private DigitalOutput red;
    private DigitalOutput green;
    private DigitalOutput blue;

    public ArduinoLED(){
        
        red = new DigitalOutput(1);
        green = new DigitalOutput(2);
        blue = new DigitalOutput(3);
        
    }

    public void setOff(){
        red.set(false);
        green.set(false);
        blue.set(false);
    }

    public void setWhite(){
        red.set(true);
        green.set(true);
        blue.set(true);
    }

    public void setRed(){
        red.set(true);
        green.set(false);
        blue.set(false);
    }

    public void setGreen(){
        red.set(false);
        green.set(true);
        blue.set(false);
    }
    
    public void setBlue(){
        red.set(false);
        green.set(false);
        blue.set(true);
    }

    





}
