
package frc.robot.util;
import java.sql.Driver;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class UltrasonicRangefinder  extends SubsystemBase{

    private AnalogInput analogInput;  
    private double scalingFactor = 0.0098;
    private double offset = 0.097;

    public UltrasonicRangefinder(LightStrip lightStrip){
       analogInput = new AnalogInput(0);
    }

    public double getDistanceMeters(){

        double analogValue = analogInput.getAverageVoltage();
        double voltage = analogValue * 5.0 / 1023.0;
        double distance = (voltage - offset) / scalingFactor;

        return distance;
    }

 
    @Override
    public void periodic() {
        SmartDashboard.putNumber("Ultrasonic Distance Meters", getDistanceMeters());
        SmartDashboard.putNumber("Ultrasonic Raw Voltage", analogInput.getAverageVoltage());

    }
}
