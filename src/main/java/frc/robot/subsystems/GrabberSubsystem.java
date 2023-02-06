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

// Ignore unused variable warnings
@SuppressWarnings("unused")


public class GrabberSubsystem extends SubsystemBase{

    private DoubleSolenoid solenod;

    // Subsystem Constructor
    public GrabberSubsystem(){
        solenod = new DoubleSolenoid(55,PneumaticsModuleType.CTREPCM, 1, 0);

        solenod.set(Value.kForward);
}

    // Update vision variables once per scheduler run
    @Override
    public void periodic(){}

    public void toggle(){
        solenod.toggle();
    }


}
