// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.util;
import java.util.ArrayList;
import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class SelfCheck extends SubsystemBase{

    SwerveSubsystem swerveSubsystem;
    GrabberSubsystem grabberSubsystem;
    ElevatorSubsystem elevatorSubsystem;
    
    private ArrayList<CANSparkMax> robotMotors = new ArrayList<>();

    public SelfCheck(SwerveSubsystem swerveSubsystem, GrabberSubsystem grabberSubsystem, ElevatorSubsystem elevatorSubsystem){

        this.swerveSubsystem = swerveSubsystem;
        this.grabberSubsystem = grabberSubsystem;
        this.elevatorSubsystem = elevatorSubsystem;


    }

    @Override
    public void periodic() {

        checkMotorFaults(robotMotors);
        checkExtensionLimits();

    }

    public void checkMotorFaults(ArrayList<CANSparkMax> robotMotors) {
    
    for (CANSparkMax motorController : robotMotors) {

        // Get motor faults as an int
        int faultFlags = motorController.getFaults();
    
        // Check for faults
        if (faultFlags != 0){

        // Check for specific faults using MotorFault enum
        for (MotorFault fault : MotorFault.values()){

            boolean hasFault = (faultFlags & fault.getValue()) != 0;

            if(hasFault){

            switch (fault) {
            case kBrownout:
                System.out.println("Spark Max: " + motorController.getDeviceId() + " is having a brownout");
                break;

            case kCanMismatch:
                System.out.println("Spark Max: " + motorController.getDeviceId() + " is having a CAN ID mismatch");
                break;

            case kStall:
                System.out.println("Spark Max: " + motorController.getDeviceId() + " is having a stall");
                break;
            
            case kSupplyVoltage:
                System.out.println("Spark Max: " + motorController.getDeviceId() + " is having a supply voltage issue");
                break;

            default:
                System.out.println("Spark Max: " + motorController.getDeviceId() + " is having a fault: " + fault.name());
                break;

            }}
        }}
    }}

    public void checkExtensionLimits(){
        if(grabberSubsystem.getAngleSetpoint() < 0){

        }
    }

    
}
