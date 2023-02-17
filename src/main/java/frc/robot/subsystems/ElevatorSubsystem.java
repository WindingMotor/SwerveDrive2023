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
    private DoubleSolenoid grabberSolenoid;
    private DoubleSolenoid leftSolenoid;
    private DoubleSolenoid rightSolenoid;

    // Motors
    private CANSparkMax motorOne;
    private CANSparkMax motorTwo;
    private CANSparkMax intakeMotor;

    private RelativeEncoder motorOneEncoder;
    private RelativeEncoder motorTwoEncoder;

    private SparkMaxPIDController elevatorPID;

    private boolean toggled;


    // Lift Subsystem Constructor
    public ElevatorSubsystem(){

        // Set solenoid object values
        grabberSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, ElevatorConstants.kGrabberSolenoidPort,  ElevatorConstants.kGrabberSolenoidPortOFF);
        leftSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, ElevatorConstants.kGrabberSolenoidPort,  ElevatorConstants.kGrabberSolenoidPortOFF);
        rightSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, ElevatorConstants.kGrabberSolenoidPort,  ElevatorConstants.kGrabberSolenoidPortOFF);

        // Set default state of solenoids
        grabberSolenoid.set(Value.kForward);
        leftSolenoid.set(Value.kForward);
        rightSolenoid.set(Value.kForward);

        // Set motor object values take in CAN ID
        motorOne = new CANSparkMax(ElevatorConstants.kLiftMotorOnePort, MotorType.kBrushless);
        motorTwo = new CANSparkMax(ElevatorConstants.kLiftMotorTwoPort, MotorType.kBrushless);
        intakeMotor = new CANSparkMax(ElevatorConstants.kIntakeMotorPort, MotorType.kBrushless);

        // Make motor two follow motor one
        motorTwo.follow(motorOne);

        // Set motor encoder position factors to meters
        motorOneEncoder.setPositionConversionFactor(0.00378485654);
        motorTwoEncoder.setPositionConversionFactor(0.00378485654);

        // Set default encoder values
        motorOneEncoder = motorOne.getEncoder();
        motorTwoEncoder = motorTwo.getEncoder();

        // WARNING: WE MIGHT HAVE TO INVERT MOTOR 2 <--

        // Set PID to motor one
        elevatorPID = motorOne.getPIDController();

        // Set default PID values
        setPIDValues(elevatorPID);

        toggled = false;
}

    private void setPIDValues(SparkMaxPIDController pid){
        // Set PID default values. ¡ I took these from the Smart Motion example !
        pid.setP(5e-5);
        pid.setI(1e-6);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF( 0.000156); 
        pid.setOutputRange(-1, 1);

        // Set max and min Smart Motion values
        pid.setSmartMotionMaxVelocity(800, 0); // RPM/s def 2000
        pid.setSmartMotionMinOutputVelocity(0, 0); 
        pid.setSmartMotionMaxAccel(500, 0); // RPM/s def 1500
        pid.setSmartMotionAllowedClosedLoopError(0,0);
    
    }

    @Override
    public void periodic(){
        updateSmartDashboard();
    }

    public void toggleGrabberSolenoid(){
        grabberSolenoid.toggle();
    }

    public void toggleIntake(){
        if(toggled){
            intakeMotor.set(0);
            toggled = false;
        }else{
            intakeMotor.set(1);
            toggled = true;
        }
    }

    public void setIntake(double x){
        intakeMotor.set(x);
    }

    public void toggleElevatorSolenoids(){
        leftSolenoid.toggle();
        rightSolenoid.toggle();
    }

    public void setElevatorSmartMotion(double x){
        elevatorPID.setReference(x, CANSparkMax.ControlType.kSmartMotion);;
    }

    public void setElevatorVelocity(double x){
        elevatorPID.setReference(x, CANSparkMax.ControlType.kVelocity);
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Elevator Encoder:", motorOneEncoder.getPosition());
    }


}
