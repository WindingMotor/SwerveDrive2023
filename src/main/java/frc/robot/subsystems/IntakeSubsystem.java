// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.GrabberConstants;
import frc.robot.util.Constants.VisionConstants;
import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
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

public class IntakeSubsystem extends SubsystemBase{

    // Solenoid
    // Intake motor
    private CANSparkMax intakeMotor;
    private RelativeEncoder intakeMotorEncoder;

    // Lift Subsystem Constructor
    public IntakeSubsystem(){

        // Set motor object values takes in CAN ID
        intakeMotor = new CANSparkMax(GrabberConstants.kIntakeMotorPort, MotorType.kBrushless);

        // Set motors to brake mode
        intakeMotor.setIdleMode(IdleMode.kCoast);
        intakeMotorEncoder = intakeMotor.getEncoder();


}

//---------------------// Periodic loop

    @Override
    public void periodic(){
        // Update smartdashboard
        updateSmartDashboard();

    }

//---------------------// Intake


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


//---------------------// Smartdashboard

    public void updateSmartDashboard(){
        SmartDashboard.putNumber("Intake Encoder:", intakeMotorEncoder.getPosition());
    }


}
