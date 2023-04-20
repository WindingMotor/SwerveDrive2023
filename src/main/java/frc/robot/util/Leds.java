// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
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

    // Modes for strobing and setting full white
    private boolean strobe, white;

    // Mode for when auto align is on
    private boolean autoAlign;

    private boolean isBalanced;

    // Mode for when SelfCheck detects something wrong
    private boolean selfcheck;

    public Leds(){

        powerDistribution = new PowerDistribution(1, ModuleType.kRev);

        cone = false; cube = false;
        endgame = false; battery = false;

        strobe = false; white = false;

        isBalanced = false;

        leds = new AddressableLED(0);
        buffer = new AddressableLEDBuffer(109);

        leds.setLength(buffer.getLength());

        leds.setData(buffer);

       // setStripColor();
        leds.start();

    }

    public void setGameObject(boolean isCone){
        
        if(isCone){
            cone = true;
            cube = false;
        }
        else{
            cube = true;
            cone = false;
        }

    }

    public void setStrobe(Boolean value){
        if(value){strobe = true;}
        else{strobe = false;}
    }

    public void setWhite(Boolean value){
        if(value){white = true;}
        else{white = false;}
    }

    public void setAutoAlign(Boolean value){
        if(value){autoAlign = true;}
        else{autoAlign = false;}
    }

    public void setAutoBalance(Boolean value){
        if(value){isBalanced = true;}
        else{isBalanced = false;}
    }

    @Override
    public void periodic() {


        //SmartDashboard.putString("LEDS", buffer.toString());

       // setStripColor();

       // solid(Section.kAll, LedColor.kCyan);
       // leds.setData(buffer);

        
        // Check if the robot is enabled
        if(DriverStation.isEnabled()){

        // Set color to desired game object for human player
        if(cone){strobe(Section.kAll, LedColor.kYellow, 0.75);}
        if(cube){strobe(Section.kAll, LedColor.kPurple, 0.75);}

        // Strobe or set leds to full white
        if(strobe){strobe(Section.kAll, LedColor.kWhite, 0.05);}
        if(white){solid(Section.kAll, LedColor.kWhite);}

        // Flash top leds green during endgame
        double matchTime =  DriverStation.getMatchTime();
        if(matchTime <= 15.0 && matchTime > 0){endgame = true;} else{endgame = false;}
        if(endgame){strobe(Section.kTopLeft, LedColor.kGreen, 0.22);
                    strobe(Section.kTopRight, LedColor.kGreen, 0.22);}
        

        // Flash bottom leds orange is battery voltage is low
        if(powerDistribution.getVoltage() < 10){battery = true;} else{battery = false;}
        if(battery){strobe(Section.kBottomLeft, LedColor.kOrange, 0.15);
                    strobe(Section.kBottomRight, LedColor.kOrange, 0.15);}

        if(autoAlign){
        solid(Section.kTopLeft, LedColor.kCyan);
        solid(Section.kTopRight, LedColor.kCyan);
        if(DriverStation.getAlliance() == Alliance.Red){
            solid(Section.kBottomLeft, LedColor.kRed);
            solid(Section.kBottomRight, LedColor.kRed);
        }else{
            solid(Section.kBottomLeft, LedColor.kBlue);
            solid(Section.kBottomRight, LedColor.kBlue);    
            }

        }

        if(DriverStation.isAutonomous()){
            if(isBalanced){
                breath(Section.kAll, LedColor.kBlue, LedColor.kBlack, 2.0);
            }else{
                breath(Section.kAll, LedColor.kRed, LedColor.kBlack, 2.0);
            }
        }
            

        }else if(DriverStation.isEStopped()){breath(Section.kAll, LedColor.kRed, LedColor.kBlack, 0.25);}  // E-Stopped Red LEDS
        else{
            // Disabled LEDS
            rainbow(Section.kAll, 50.0, 1);
            breath(Section.kTopLeft, LedColor.kRed, LedColor.kGreen, 1.0);
            breath(Section.kTopRight, LedColor.kGreen, LedColor.kRed, 1.0);
        }   

        // Update the leds to the buffer
        leds.setData(buffer);
        
    }


    // Make an entire section a selected color
    public void solid(Section section, LedColor color){
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
    public void solid(Section section){
        for(int i = section.getStart(); i < section.getEnd(); i++){
            buffer.setLED(i, LedColor.kBlack.getColor());
        }
    }

    // Strobe a color to a section 
    private void strobe(Section section, LedColor color, double seconds){
        // Strobing timing
        boolean isOn = ((Timer.getFPGATimestamp() % seconds) / seconds) > 0.5;
        // Display color if isOn is true
        solid(section, isOn ? color : LedColor.kBlack);

    }

    // Apply a rainbow effect to a section
    private void rainbow(Section section, double cycle, double seconds){
        double xS = (1 - ((Timer.getFPGATimestamp() / seconds) % 1.0)) * 180.0;
        double xD = 180.0 / cycle;
        for(int i = 0; i < section.getEnd(); i++){
            xS += xD; xS %= 180;
            if(i >= section.getStart()){buffer.setHSV(i, (int) xS, 255, 255);}
        }
    }
    
    // Fade a section between two specified colors
    private void breath(Section section, LedColor c1, LedColor c2, double seconds){
        double xS = ((Timer.getFPGATimestamp() % seconds) / seconds) * 2.0 * Math.PI;
        double ratio = (Math.sin(xS) + 1.0) / 2.0;
        double r = (c1.getRed() * (1 - ratio)) + (c2.getRed() * ratio);
        double g = (c1.getGreen() * (1 - ratio)) + (c2.getGreen() * ratio);
        double b = (c1.getBlue() * (1 - ratio)) + (c2.getBlue() * ratio);
        solid(section, new int[]{(int)r,(int)g,(int)b});
    }




}
