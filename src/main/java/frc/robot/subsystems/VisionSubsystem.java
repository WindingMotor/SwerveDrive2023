// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.VisionConstants;
import org.photonvision.PhotonCamera;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Ignore unused variable warnings
@SuppressWarnings("unused")


public class VisionSubsystem extends SubsystemBase{
    
    // Create a vision photon camera
    PhotonCamera visionCamera;

    // Camera result for vision camera
    private PhotonPipelineResult cameraResult;

    // Subsystem Constructor
    public VisionSubsystem(){
        
        // Port forward photon vision so we can access it with an ethernet cable
        PortForwarder.add(5800, "photonvision.local", 5800);

        // Set object values to camera
        visionCamera = new PhotonCamera("camera name goes here");

        // Update camera results before periodic
        updateCameraResults();
    }

    // Update the camera results
    private void updateCameraResults(){
        cameraResult = visionCamera.getLatestResult();
    }

    // Checks if camera sees targets, must use to stop null exeptions!
    private Boolean hasTargets(){
        return(cameraResult.hasTargets());
    }

    // Returns the best target from the camera
    private PhotonTrackedTarget getBestTarget(){
        return(cameraResult.getBestTarget());
    }

    // Returns a percentage of how much area a target takes up, 0 - 100 percent
    private double getTargetArea(){
        return(getBestTarget().getArea());
    }

    // Returns the april tag ID number
    private int getTargetID(){
        return(getBestTarget().getFiducialId());
    }


    // Update the smart dashboard
    private void updateSmartDashboard(){
        // Put targets? value
        SmartDashboard.putBoolean("Targets?", hasTargets());
        // Check if targets are found before putting values to prevent null!
        if(hasTargets()){
            SmartDashboard.putString("Target ID", getTargetID() + "");
            SmartDashboard.putString("Target Area", getTargetArea() + "%");
        }else{
            SmartDashboard.putString("Target ID", "No ID Found!");
            SmartDashboard.putString("Target Area", "0" + "%");
        }
        
    }

    // A periodic loop, updates smartdashboard and camera results
    @Override
    public void periodic(){
        updateCameraResults();
        updateSmartDashboard();
    }


}
