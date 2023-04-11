// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicRangefinder extends SubsystemBase{

    private AnalogInput analogInput;  
   // private double scalingFactor = 0.0098;
   // private double offset = 0.097;

    public UltrasonicRangefinder(LightStrip lightStrip){
       analogInput = new AnalogInput(3);
    }

    public double getDistanceMM(){

        double analogValue = analogInput.getVoltage();
        double distance = analogValue * 5;
        return distance;

    }

    public double getDistanceM(){
        return(getDistanceMM() / 1000);
    }

    public boolean isDistanceMaxMin(double max, double min){
        if(getDistanceM() < max && getDistanceM() > min){
            return true;
        }
        return false;
    }

    public boolean isDistanceMax(double max){
        if(getDistanceM() > max){
            return true;
        }
        return false;
    }

    public boolean isDistanceMin(double min){
        if(getDistanceM() < min){
            return true;
        }
        return false;
    }

    public boolean isLoadingWall(){
        return(isDistanceMaxMin(0.5, 0.25)); 
    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ultrasonic Distance Meters", getDistanceM());
        SmartDashboard.putNumber("Ultrasonic Distance Millimeters", getDistanceMM());
        SmartDashboard.putNumber("Ultrasonic Raw Voltage", analogInput.getVoltage());

        double voltage = analogInput.getVoltage();
        double distance = voltage / 0.0098; //convert voltage to distance using the formula in the datasheet

        SmartDashboard.putNumber("Ultrasonic Raw Inches", distance);

    }
}
