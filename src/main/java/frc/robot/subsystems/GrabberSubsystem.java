// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.ElevatorConstants;
import frc.robot.util.Constants.VisionConstants;

import com.revrobotics.AbsoluteEncoder;
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

public class GrabberSubsystem extends SubsystemBase{

    // Solenoids
    private DoubleSolenoid grabberSolenoid;

    private CANSparkMax intakeMotor;
    private CANSparkMax angleMotor;

    private SparkMaxPIDController anglePID;

    private RelativeEncoder angleMotorEncoder;

    private boolean toggled;


    // Lift Subsystem Constructor
    public GrabberSubsystem(){

        // Set solenoid object values
        grabberSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, ElevatorConstants.kGrabberSolenoidPort,  ElevatorConstants.kGrabberSolenoidPortOFF);

        // Set default state of solenoid
        grabberSolenoid.set(Value.kForward);

        // Set motor object values takes in CAN ID
        intakeMotor = new CANSparkMax(ElevatorConstants.kIntakeMotorPort, MotorType.kBrushless);

        angleMotorEncoder = angleMotor.getEncoder();

        // Set default PID values
        setMotorPID(anglePID);

        toggled = false;
}

    private void setMotorPID(SparkMaxPIDController pid){
        // Set PID default values. ¡ I took these from the Smart Motion example !
        pid.setP(5e-5);
        pid.setI(1e-6);
        pid.setD(0);
        pid.setIZone(0);
        pid.setFF( 0.000156); 
        pid.setOutputRange(-1, 1);

        // Set max and min Smart Motion values
        pid.setSmartMotionMaxVelocity(1000, 0); // RPM/s def 2000
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

    public void setIntakeSpeed(double x){
        intakeMotor.set(x);
    }

    public void setAngleSmartMotion(double x){
        anglePID.setReference(x, CANSparkMax.ControlType.kSmartMotion);;
    }

    public void setAngleVelocity(double x){
        anglePID.setReference(x, CANSparkMax.ControlType.kVelocity);
    }

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Grabber Encoder:", angleMotorEncoder.getPosition());
    }


}
