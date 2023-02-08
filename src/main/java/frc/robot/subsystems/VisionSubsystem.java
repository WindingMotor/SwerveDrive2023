// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.VisionConstants;
import java.util.List;
import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.common.hardware.VisionLEDMode;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.net.PortForwarder;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
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
      //  PortForwarder.add(5800, "photonvision.local", 5800);

        // Set object values to camera
        visionCamera = new PhotonCamera("OV5647");

        // Update camera results before periodic
        updateCameraResults();

        visionCamera.setLED(VisionLEDMode.kDefault);
        
    }

    // Update the camera results
    private void updateCameraResults(){
        cameraResult = visionCamera.getLatestResult();
    }

    // Checks if camera sees targets, must use to stop null exeptions!
    private Boolean hasTargets(){
        return(cameraResult.hasTargets());
    }

    // Returns the single best target from the camera
    private PhotonTrackedTarget getBestTarget(){
        return(cameraResult.getBestTarget());
    }

    // Returns all targets from the camera in an array
    private List<PhotonTrackedTarget> getTargetList(){
        return(cameraResult.getTargets());
    }

    // Returns a picked target from the target list
    private PhotonTrackedTarget getTargetFromList(int num){
        return(cameraResult.getTargets().get(num));
    }

    // Returns the size of the target list
    private int getListSize(){
        return(cameraResult.getTargets().size());
    }

    // Returns a percentage of how much area a target takes up, 0 - 100 percent
    private double getTargetArea(){
        return(getBestTarget().getArea());
    }

    private double getTargetPitch(){
        return(getBestTarget().getPitch());
    }

    // Returns the april tag ID number
    private int getTargetID(){
        return(getBestTarget().getFiducialId());
    }

    private double getTargetDistance(){
        // Camera height M, Target height M, Camera pitch R, Target pitch R.
         return(PhotonUtils.calculateDistanceToTargetMeters(0.774, 0.361,
         0.0174, getTargetPitch()));
    }

    public Transform3d getTargetTransform(){
        // Camera height M, Target height M, Camera pitch R, Target pitch R.
         return(getBestTarget().getBestCameraToTarget());
    }


    // Update the smart dashboard
    private void updateSmartDashboard(){
        // Put targets? value
        SmartDashboard.putBoolean("Targets?", hasTargets());
        // Check if targets are found before putting values to prevent null!
        if(hasTargets()){
            SmartDashboard.putString("Target ID", getTargetID() + "");
            SmartDashboard.putString("Target Pitch", getTargetPitch() + "");
            SmartDashboard.putString("Target Area", getTargetArea() + "%");
            SmartDashboard.putNumber("Target Distance X-Plane", getTargetTransform().getX());
        }else{
            SmartDashboard.putString("Target ID", "No ID Found!");
            SmartDashboard.putString("Target Pitch", "-1");
            SmartDashboard.putString("Target Area", "0" + "%");
            SmartDashboard.putNumber("Target Distance X-Plane", -1);
        }
        SmartDashboard.putString("LED State", visionCamera.getLEDMode().toString());
        
    }

    public void setLEDOn(){
        visionCamera.setLED(VisionLEDMode.kBlink);
        DriverStation.reportWarning("CHANGE LED", true);

    }

    // A periodic loop, updates smartdashboard and camera results
    @Override
    public void periodic(){
        //SmartDashboard.putString("Camera", visionCamera.toString());
        updateCameraResults();
        updateSmartDashboard();

    }


}
