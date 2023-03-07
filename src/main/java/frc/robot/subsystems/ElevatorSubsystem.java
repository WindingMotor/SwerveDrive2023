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
import edu.wpi.first.math.filter.SlewRateLimiter;
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

    // Solenoid
    private DoubleSolenoid solenoid;

    // Motors
    private CANSparkMax motorOne;
    private CANSparkMax motorTwo;

    // Encoder and PIDf
    private RelativeEncoder motorOneEncoder;
    private PIDController elevatorPID;

    // Limit switch
    private SparkMaxLimitSwitch bottomLimitSwitch;

    // Elevator Setpoint
    private double elevatorSetpointMeters;

    // Elevator Change of speed limiter
    private SlewRateLimiter slewRateLimiter;

    // Lift Subsystem Constructor
    public ElevatorSubsystem(){

        // Set solenoid object values
        solenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, ElevatorConstants.kSolenoidPort,  ElevatorConstants.kSolenoidPortOFF);

        // Set default state of solenoid
        solenoid.set(Value.kForward);

        // Set motor object values take in CAN ID
        motorOne = new CANSparkMax(ElevatorConstants.kElevatorMotorOnePort, MotorType.kBrushless);
        motorTwo = new CANSparkMax(ElevatorConstants.kElevatorMotorTwoPort, MotorType.kBrushless);

        // Make motor two follow motor one
        motorTwo.follow(motorOne);

        // Set motors to brake mode
        motorOne.setIdleMode(IdleMode.kBrake);
        motorTwo.setIdleMode(IdleMode.kBrake);

        // Set default encoder values
        motorOneEncoder = motorOne.getEncoder();

        // Set motor encoder position factors to meters
        motorOneEncoder.setPositionConversionFactor(0.02367145);

        //0.0066509 * 4
        // Get and set bottom limit switch
        bottomLimitSwitch = motorOne.getForwardLimitSwitch(SparkMaxLimitSwitch.Type.kNormallyOpen);
        bottomLimitSwitch.enableLimitSwitch(false);

        // Set PID values for elevator
        //elevatorPID = new PIDController(0.5, 0, 0);
        elevatorPID = new PIDController(0.7, 0.05, 0);

        // Set slew rate limiter max and min rates
        slewRateLimiter = new SlewRateLimiter(5, 10, 0);
        
        // Set setpoint to zero
        elevatorSetpointMeters = 0;
    }

    @Override
    public void periodic(){
        updateSmartDashboard();
        updateElevatorMeters();
        SmartDashboard.putNumber("Elevator Setpoint", elevatorSetpointMeters);
        
    }

    public void toggleElevatorSolenoid(){
        solenoid.toggle();
    }

    public void updateElevatorSetpoint(double input){
        // Update the elevator setpoint in meters with joystick input with -1 to +1
       // input /= 2;
       // input = Math.abs(input) > 0.05 ? input : 0.0;
        elevatorSetpointMeters = input * 1.4;
    }

    public void setElevatorSetpoint(double input){
        // Update the elevator setpoint in meters with joystick input with -1 to +1
        elevatorSetpointMeters = input;
    }

    public void addElevatorSetpoint(double input){
        // Update the elevator setpoint in meters with joystick input with -1 to +1
        elevatorSetpointMeters += input;
    }

    public void subtractElevatorSetpoint(double input){
        // Update the elevator setpoint in meters with joystick input with -1 to +1
        elevatorSetpointMeters -= input;
    }

    private void updateElevatorMeters(){

        // Takes in current elevator position in meters and the setpoint in meters and outputs change needed
        double caculated = elevatorPID.calculate(motorOneEncoder.getPosition(), elevatorSetpointMeters);

        // Apply slew to caculated output
        caculated = slewRateLimiter.calculate(caculated);

        // Set motors to caculated value
        motorOne.set(caculated);

    }

    // Set both elevator motors to input
    public void setElevatorMotors(double x){
        motorOne.set(x); 
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
       // SmartDashboard.putBoolean("Bottom Limit Switch", bottomLimitSwitch.isPressed());
    }


}
