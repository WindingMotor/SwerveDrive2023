// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.VisionConstants;
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

public class LiftSubsystem extends SubsystemBase{

    // Solenoids
    private DoubleSolenoid intakeSolenoid;
    // private DoubleSolenoid leftSolenoid;
    // private DoubleSolenoid rightSolenoid;

    private Boolean intakeSolenoidState;
    private Boolean liftSolenoidState;
    

    // Lift Subsystem Constructor
    public LiftSubsystem(){

        // Set solenoid object values
        intakeSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, 1, 0);
      //  leftSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, 1, 0);
  //      rightSolenoid = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, 1, 0);

        // Set default state of solenoids
        intakeSolenoid.set(Value.kForward);
       // leftSolenoid.set(Value.kForward);
       // rightSolenoid.set(Value.kForward);

        // Set default state of solenoid states
        intakeSolenoidState = false;
        liftSolenoidState = false;
}

    @Override
    public void periodic(){

        // Update the smartdashboard
        //updateSmartDashboard();
    }

    public void updateSmartDashboard(){

          // Put solenoid states on smartdashboard
        //  SmartDashboard.putBoolean("Intake Solenoid", intakeSolenoidState);
        //  SmartDashboard.putBoolean("Lift Solenoids", liftSolenoidState);
  
    }

    public void toggleIntakeSolenoid(){
        // Toggle the solenoid and update solenoid state variable
        intakeSolenoid.toggle();
      //  if(intakeSolenoidState){intakeSolenoidState = false;}
       // else{intakeSolenoidState = true;}
    }



}
