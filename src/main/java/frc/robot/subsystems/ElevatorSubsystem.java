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
    private CANSparkMax MotorOne;
    private CANSparkMax MotorTwo;

    private RelativeEncoder MotorOneEncoder;
    private RelativeEncoder MotorTwoEncoder;

    private SparkMaxPIDController ElevatorPID;

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
        MotorOne = new CANSparkMax(ElevatorConstants.kLiftMotorOnePort, MotorType.kBrushless);
        MotorTwo = new CANSparkMax(ElevatorConstants.kLiftMotorTwoPort, MotorType.kBrushless);

        // Make motor two follow motor one
        MotorTwo.follow(MotorOne);

        // Set motor encoder position factors to meters
        MotorOneEncoder.setPositionConversionFactor(0.00378485654);
        MotorTwoEncoder.setPositionConversionFactor(0.00378485654);

        // Set default encoder values
        MotorOneEncoder = MotorOne.getEncoder();
        MotorTwoEncoder = MotorTwo.getEncoder();

        // Set PID to motor one
        ElevatorPID = MotorOne.getPIDController();

        // Set default PID values
        setPIDValues(ElevatorPID);

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
    }

    public void toggleGrabberSolenoid(){
        grabberSolenoid.toggle();
    }

    public void toggleElevatorSolenoids(){
        leftSolenoid.toggle();
        rightSolenoid.toggle();
    }

    public void setElevatorSetpoint(double x){
        ElevatorPID.setReference(x, CANSparkMax.ControlType.kSmartMotion);;
        SmartDashboard.putNumber("Report Velocity", MotorOneEncoder.getPosition());
    }

    public void setElevatorVelocity(double x){
        ElevatorPID.setReference(x, CANSparkMax.ControlType.kVelocity);
        SmartDashboard.putNumber("Report Velocity", MotorOneEncoder.getPosition());
    }



}
