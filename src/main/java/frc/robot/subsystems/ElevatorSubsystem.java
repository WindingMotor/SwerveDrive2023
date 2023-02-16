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

    private SparkMaxPIDController PIDOne;
    private SparkMaxPIDController PIDTwo;

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

        MotorOne = new CANSparkMax(ElevatorConstants.kLiftMotorOnePort, MotorType.kBrushless);
        MotorTwo = new CANSparkMax(ElevatorConstants.kLiftMotorTwoPort, MotorType.kBrushless);

        MotorOneEncoder = MotorOne.getEncoder();
        MotorTwoEncoder = MotorTwo.getEncoder();

        PIDOne = MotorOne.getPIDController();
        PIDTwo = MotorTwo.getPIDController();

        setPIDValues(PIDOne);
        setPIDValues(PIDTwo);
}

    private void setPIDValues(SparkMaxPIDController pid){
        pid.setP(5e-5);
        pid.setI(1e-6);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF( 0.000156); 
        pid.setOutputRange(-1, 1);


        pid.setSmartMotionMaxVelocity(2000, 0); // RPM
        pid.setSmartMotionMinOutputVelocity(0, 0); 
        pid.setSmartMotionMaxAccel(1500, 0);
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
        double[] report = {0,0};
        PIDOne.setReference(x, CANSparkMax.ControlType.kSmartMotion);
        PIDTwo.setReference(x, CANSparkMax.ControlType.kSmartMotion);
        report[0] = MotorOneEncoder.getPosition();
        report[1] = MotorTwoEncoder.getPosition();
        SmartDashboard.putString("Report Position", "One:" + report[0] + " Two: " + report[1]);

    }

    public void setElevatorVelocity(double x){
        double[] report = {0,0};
        PIDOne.setReference(x, CANSparkMax.ControlType.kVelocity);
        PIDTwo.setReference(x, CANSparkMax.ControlType.kVelocity);
        report[0] = MotorOneEncoder.getPosition();
        report[1] = MotorTwoEncoder.getPosition();
        SmartDashboard.putString("Report Velocity", "One:" + report[0] + " Two: " + report[1]);
    }



}
