// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;

// Ignore unused variable warnings
@SuppressWarnings("unused")
    public class GrabberSubsystem extends SubsystemBase{

        
    private Spark grab = new Spark(0);

    // Subsystem Constructor
    public GrabberSubsystem(){}

    @Override
    public void periodic(){}

    public void close(double s){
        grab.set(s);
    }
    
      public void open(double s){
        grab.set(-s);
    }
    


}
