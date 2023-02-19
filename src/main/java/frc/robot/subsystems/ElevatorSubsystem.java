// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.VisionConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class ElevatorSubsystem extends SubsystemBase{

    // Solenoids
    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;

    // Motors
    private CANSparkMax motorOne;
    private CANSparkMax motorTwo;

    private RelativeEncoder motorOneEncoder;
    private RelativeEncoder motorTwoEncoder;

    // Lift Subsystem Constructor
    public ElevatorSubsystem(){

        // Set solenoid object values
        leftSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, ElevatorConstants.kGrabberSolenoidPort,  ElevatorConstants.kGrabberSolenoidPortOFF);
        rightSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, ElevatorConstants.kGrabberSolenoidPort,  ElevatorConstants.kGrabberSolenoidPortOFF);

        // Set default state of solenoids
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);

        // Set motor object values take in CAN ID
        motorOne = new CANSparkMax(ElevatorConstants.kLiftMotorOnePort, MotorType.kBrushless);
        motorTwo = new CANSparkMax(ElevatorConstants.kLiftMotorTwoPort, MotorType.kBrushless);

        // Make motor two follow motor one
        motorTwo.follow(motorOne);

        // Set motor encoder position factors to meters
        motorOneEncoder.setPositionConversionFactor(0.00378485654);
        motorTwoEncoder.setPositionConversionFactor(0.00378485654);

        // Set default encoder values
        motorOneEncoder = motorOne.getEncoder();
        motorTwoEncoder = motorTwo.getEncoder();

        // WARNING: WE MIGHT HAVE TO INVERT MOTOR 2 <--

}


    @Override
    public void periodic(){
        updateSmartDashboard();
    }

    public void toggleElevatorSolenoids(){
        leftSolenoid.toggle();
        rightSolenoid.toggle();
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Elevator Encoder:", motorOneEncoder.getPosition());
    }


}
