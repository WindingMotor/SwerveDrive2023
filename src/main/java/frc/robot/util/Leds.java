// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.SubsystemBase;


public class Leds  extends SubsystemBase{
    
    // Amount of LEDS
    private final int length = 109;

    // Data port of roboRio connect to LEDS
    private final int port = 0;

    // Led and its buffer
    private AddressableLED leds;
    private AddressableLEDBuffer buffer;

    private PowerDistribution powerDistribution;

    // Human player cone and cube display
    private boolean cone, cube;

    // Warnings for engame timer and battery voltage
    private boolean endgame, battery;

    public Leds(){

        leds = new AddressableLED(port);
        buffer = new AddressableLEDBuffer(length);

        powerDistribution = new PowerDistribution(1, ModuleType.kRev);

        cone = false; cube = false;
        endgame = false; battery = false;

    }

    @Override
    public synchronized void periodic() {

        // Check if the robot is enabled
        if(DriverStation.isEnabled()){

        // Set color to desired game object for human player
        if(cone){solid(Section.kAll, LedColor.kYellow);}
        else if(cube){solid(Section.kAll, LedColor.kPurple);}

    
        // Flash top leds during endgame
        double matchTime =  DriverStation.getMatchTime();
        if(matchTime <= 15.0 && matchTime >= 0){endgame = true;} else{endgame = false;}
        if(endgame){strobe(Section.kTopLeft, LedColor.kRed, 0.15);
                    strobe(Section.kTopRight, LedColor.kRed, 0.15);}
        

        // Flash bottom leds is battery voltage is low
        if(powerDistribution.getVoltage() < 10){battery = true;} else{battery = false;}
        if(battery){strobe(Section.kBottomLeft, LedColor.kOrange, 0.1);
                    strobe(Section.kBottomRight, LedColor.kOrange, 0.1);}


        }else if(
            // E-Stopped Red LEDS
            DriverStation.isEStopped()){solid(Section.kAll, LedColor.kRed);} 
        else{
            // Disabled LEDS
            rainbow(Section.kAll, 25.0, 0.25);
            breath(Section.kTopLeft, LedColor.kRed, LedColor.kBlack, 1.0);
            breath(Section.kTopRight, LedColor.kGreen, LedColor.kBlack, 1.0);
        }   

        // Update the leds to the buffer
        leds.setData(buffer);

    }

    // Make an entire section a selected color
    private void solid(Section section, LedColor color){
        for(int i = section.getStart(); i < section.getEnd(); i++){
            buffer.setLED(i, color.getColor());
        }
    }

    // Make an entire a color with raw rgb values
    private void solid(Section section, int[] rgb){
        for(int i = section.getStart(); i < section.getEnd(); i++){
            buffer.setRGB(i, rgb[0], rgb[1], rgb[2]);
        }
    }

    // Make an entire section black
    private void solid(Section section){
        for(int i = section.getStart(); i < section.getEnd(); i++){
            buffer.setLED(i, LedColor.kBlack.getColor());
        }
    }

    private void strobe(Section section, LedColor color, double seconds){
        // Strobing timing
        boolean isOn = ((Timer.getFPGATimestamp() % seconds) / seconds) > 0.5;
        // Display color if isOn is true
        solid(section, isOn ? color : LedColor.kBlack);

    }

    private void rainbow(Section section, double cycle, double seconds){
        double xS = (1 - ((Timer.getFPGATimestamp() / seconds) % 1.0)) * 180.0;
        double xD = 180.0 / cycle;
        for(int i = 0; i < section.getEnd(); i++){
            xS += xD; xS %= 180;
            if(i >= section.getStart()){buffer.setHSV(i, (int) xS, 255, 255);}
        }
    }
    
    private void breath(Section section, LedColor c1, LedColor c2, double seconds){
        double xS = ((Timer.getFPGATimestamp() % seconds) / seconds) * 2.0 * Math.PI;
        double ratio = (Math.sin(xS) + 1.0) / 2.0;
        double r = (c1.getRed() * (1 - ratio)) + (c2.getRed() * ratio);
        double g = (c1.getGreen() * (1 - ratio)) + (c2.getGreen() * ratio);
        double b = (c1.getBlue() * (1 - ratio)) + (c2.getBlue() * ratio);
        solid(section, new int[]{(int)r,(int)g,(int)b});
    }




}
