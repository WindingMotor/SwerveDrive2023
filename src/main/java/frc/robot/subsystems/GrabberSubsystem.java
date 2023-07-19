// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.GrabberConstants;
import frc.robot.util.Constants.VisionConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.FaultID;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.math.controller.PIDController;
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

    // Solenoid
    private DoubleSolenoid grabberSolenoid;

    // Intake motor
    public CANSparkMax intakeMotor;
    private RelativeEncoder intakeMotorEncoder;

    // Angle motor 
    private CANSparkMax angleMotor;
    private RelativeEncoder angleMotorEncoder;
    private PIDController anglePID;
    private double angleSetpoint;

    // Solenoid state 
    private boolean grabberOpen;

    // Lift Subsystem Constructor
    public GrabberSubsystem(){

        // Set solenoid object values
        grabberSolenoid = new DoubleSolenoid(PneumaticsModuleType.REVPH, GrabberConstants.kGrabberSolenoidPort,  GrabberConstants.kGrabberSolenoidPortOFF);

        // Set default state of solenoid
        grabberSolenoid.set(Value.kForward);

        // Set motor object values takes in CAN ID
        intakeMotor = new CANSparkMax(GrabberConstants.kIntakeMotorPort, MotorType.kBrushless);
        angleMotor = new CANSparkMax(GrabberConstants.kAngleMotorPort, MotorType.kBrushless);

        // Set motors to brake mode
        intakeMotor.setIdleMode(IdleMode.kBrake);
        angleMotor.setIdleMode(IdleMode.kBrake);

        // Set angle encoder to angle motor 
        angleMotorEncoder = angleMotor.getEncoder();
        intakeMotorEncoder = intakeMotor.getEncoder();
        
        // Encoder to degrees converison factor
        angleMotorEncoder.setPositionConversionFactor(360 / 71.5982);

        // Set PID controller values
        anglePID = new PIDController(0, 0, 0);

        // Set defualt angle setpoint
        angleSetpoint = 0;

        grabberOpen = false;

        intakeMotor.setSmartCurrentLimit(5);

        angleMotor.setIdleMode(IdleMode.kBrake);
}

//---------------------// Periodic loop

    @Override
    public void periodic(){
        // Update smartdashboard
        updateSmartDashboard();
        setAnglePID(angleSetpoint);
        SmartDashboard.putBoolean("Grabber Open", grabberOpen);
    }

    public void setCurrentLimit(int amount){
        intakeMotor.setSmartCurrentLimit(amount);
    }
//---------------------// Intake

    // Open or close the intake soldenoid with a toggle
    public void toggleGrabberSolenoid(){
        grabberSolenoid.toggle();
        grabberOpen = !grabberOpen;
    }

    public boolean isGrabberOpen(){
        return(grabberOpen);
    }

    // Sets intake motor speed to 1
    public void toggleIntake(){
        intakeMotor.set(1);
    }

    public void stopIntake(){
        intakeMotor.set(0);
    }

    // Sets intake motor to a speed from 0 to 1
    public void setIntakeSpeed(double x){
        intakeMotor.set(x);
    }

    

//---------------------// Angle

    public void setAngleSpeed(double x){
        angleMotor.set(0);
    }

    public void setAngleSetpoint(double d){
        angleSetpoint = 0;
    }

    public void setAnglePID(double x){

        angleMotor.set(0);
        /* 
        if(x > 130){
            x = 95;
        }else if(x < 8){
            x = 8;
        }
        double angle = anglePID.calculate(angleMotorEncoder.getPosition(), x);
        angleMotor.set(angle);
        */
    }

//---------------------// Motor Data

    public short getAngleMotorFaults(){
        return(angleMotor.getFaults());
    }

    public double getAngleMotorCurrent(){
        return(angleMotor.getOutputCurrent());
    }


    public short getIntakeMotorFaults(){
        return(intakeMotor.getFaults());
    }

    public double getIntakeMotorCurrent(){
        return(intakeMotor.getOutputCurrent());
    }

    public double getAngleSetpoint(){
        return(angleSetpoint);
    }

    public CANSparkMax getAngleMotor(){
        return angleMotor;
    }

    public CANSparkMax getIntakeMotor(){
        return intakeMotor;
    }

//---------------------// Smartdashboard

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Grabber Encoder DEG:", angleMotorEncoder.getPosition());
        SmartDashboard.putNumber("Intake Encoder:", intakeMotorEncoder.getPosition());
    }


}
