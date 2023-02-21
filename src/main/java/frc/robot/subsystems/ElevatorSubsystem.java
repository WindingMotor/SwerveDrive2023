// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.VisionConstants;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxLimitSwitch;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.Timer;
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
    private PIDController elevatorPID;

    private SparkMaxLimitSwitch bottomLimitSwitch;


    // Lift Subsystem Constructor
    public ElevatorSubsystem(){

        // Set solenoid object values
        leftSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ElevatorConstants.kLeftSolenoidPort,  ElevatorConstants.kLeftSolenoidPortOFF);
        rightSolenoid = new DoubleSolenoid(PneumaticsModuleType.CTREPCM, ElevatorConstants.kRightSolenoidPort,  ElevatorConstants.kRightSolenoidPortOFF);

        // Set default state of solenoids
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);

        // Set motor object values take in CAN ID
        motorOne = new CANSparkMax(ElevatorConstants.kLiftMotorOnePort, MotorType.kBrushless);
        motorTwo = new CANSparkMax(ElevatorConstants.kLiftMotorTwoPort, MotorType.kBrushless);

        // Make motor two follow motor one
        motorTwo.follow(motorOne);

        // Set motor to brake mode
        motorOne.setIdleMode(IdleMode.kBrake);
        motorTwo.setIdleMode(IdleMode.kBrake);

        // Set motor encoder position factors to meters
        motorOneEncoder.setPositionConversionFactor(0.00378485654 * 2);
        motorTwoEncoder.setPositionConversionFactor(0.00378485654 * 2);

        // Set default encoder values
        motorOneEncoder = motorOne.getEncoder();
        motorTwoEncoder = motorTwo.getEncoder();

        // Get and set bottom limit switch
        bottomLimitSwitch = motorOne.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyClosed);
        bottomLimitSwitch.enableLimitSwitch(true);

        // Set PID values from constants
        elevatorPID = new PIDController(ElevatorConstants.kP, 0, 0);
    }

    @Override
    public void periodic(){
        updateSmartDashboard();
    }

    public void toggleElevatorSolenoids(){
        leftSolenoid.toggle();
        rightSolenoid.toggle();
    }

    public void setElevatorMeters(double setpoint){

        // Get motor position in meters
        double position = motorOneEncoder.getPosition();

        // Takes in current elevator position in meters and the setpoint in meters and outputs change needed
        double caculated = elevatorPID.calculate(position, setpoint);
        caculated /= 2;

        // Set motors to caculated posi
        motorOne.set(caculated);
        motorTwo.set(caculated);
    }

    // Set both elevator motors to input
    public void setElevatorMotors(double x){
        motorOne.set(x);
        motorTwo.set(x);
    }

    // Set both elevator motors to zero
    public void stopElevator(){
        setElevatorMotors(0);
    }

    /* Automatically home the elevator, bring the elevator down until it its the limit switch
    then bring it up for 0.5 seconds and then bring it back down slower for more precision */ 
    public boolean homeElevatorBottom(){ 
        if(bottomLimitSwitch.isPressed() == false){
            setElevatorMotors(-0.25);
        }else{
            setElevatorMotors(0.15);
            Timer.delay(0.5);
            if(bottomLimitSwitch.isPressed() == false){
                setElevatorMotors(-0.1);
            }else{
                return true;
            }
        }
        return true;
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Elevator Encoder:", motorOneEncoder.getPosition());
        SmartDashboard.putBoolean("Bottom Limit Switch", bottomLimitSwitch.isPressed());
    }


}
