// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

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


public class VisionSubsystem extends SubsystemBase{
    
    //PhotonCamera camera = new PhotonCamera("photonvision");

   //  public Boolean hasTargets = false;
   //  public PhotonPipelineResult result;
   // public PhotonTrackedTarget bestTarget;

    // Subsystem Constructor
    public VisionSubsystem(){
        
}

    // Update vision variables once per scheduler run
    @Override
    public void periodic(){

    // Query the latest result from PhotonVision
    // var result = camera.getLatestResult();

    // Check if the latest result has any targets.

    //boolean hasTargets = result.hasTargets();

    // Get the current best target.
    //PhotonTrackedTarget target = result.getBestTarget();

    //int targetID = target.getFiducialId();

   //     SmartDashboard.putNumber("Target ID", targetID);
       // SmartDashboard.putBoolean("Target?", hasTargets);

    }

   // public int getTargetID(){
   //    return(bestTarget.getFiducialId());
    //}

}
