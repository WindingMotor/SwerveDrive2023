// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import java.text.DecimalFormat;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Transmitter {

    // Joystick object
    // My Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2
    // My Transmitter max and min values: 0.700 and -0.700
    private Joystick joystick;
    DecimalFormat df = new DecimalFormat("###.###");

    // Class constructor takes: joystick ID
    public Transmitter(int id){
        joystick = new Joystick(id);
    }

    public double calculateOffset(double d){

        // Convert double to smaller decimal value by type conversion
        double x = d; // Double.parseDouble(df.format(d));
        
        // Return value with applied offsets depending on if negative or postitive or zero
        if(x > 0.000){return(x * Constants.IOConstants.kTransmitterOffset);}
        if(x <  0.000){return(x * Constants.IOConstants.kTransmitterOffset);}
        else{return(0.000);} 
    }

    
    public double getRoll(){
        SmartDashboard.putNumber("Roll", calculateOffset(joystick.getRawAxis(0)));
        return(calculateOffset(joystick.getRawAxis(0)));
    }

    public double getPitch(){
        SmartDashboard.putNumber("Pitch", calculateOffset(joystick.getRawAxis(1)));
        return(calculateOffset(joystick.getRawAxis(1)));
    }

    public double getThrottle(){
        SmartDashboard.putNumber("Throttle", calculateOffset(joystick.getRawAxis(2)));
        return(calculateOffset(joystick.getRawAxis(2)));
    }

    public double getYaw(){
        SmartDashboard.putNumber("Yaw", calculateOffset(joystick.getRawAxis(3)));
        return(calculateOffset(joystick.getRawAxis(3)));
    }

    public boolean getSwitchVeryRight(){
        return(joystick.getRawButton(1));
    }

    public boolean getSwitchRight(){
        return(joystick.getRawButton(1));
    }

    public boolean getSwitchVeryLeft(){
        return(joystick.getRawButton(1));
    }

    public boolean getSwitchLeft(){
        return(joystick.getRawButton(1));
    }

}
