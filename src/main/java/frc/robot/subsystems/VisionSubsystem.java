// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.VisionConstants;

import org.photonvision.PhotonCamera;
import org.photonvision.PhotonUtils;
import org.photonvision.targeting.PhotonPipelineResult;
import org.photonvision.targeting.PhotonTrackedTarget;

import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// Ignore unused variable warnings
@SuppressWarnings("unused")
    public class VisionSubsystem extends SubsystemBase{
    
    private PhotonCamera camera;
    public PhotonTrackedTarget target;
    public boolean hasTargets = false;
    public PhotonPipelineResult result;

    // Subsystem Constructor
    public VisionSubsystem(){

      camera = new PhotonCamera("2106camera");
    }

    @Override
    public void periodic(){

      // A container holding data of the photon camera result
      result = camera.getLatestResult();

      // Check if camera has targets, must be called!
      hasTargets = result.hasTargets();

      // Get the "best" target
      target = result.getBestTarget();

      SmartDashboard.putBoolean("Targets?", hasTargets);
      SmartDashboard.putNumber("Target ID", getTargetID());
      SmartDashboard.putString("Target Transform", getTargetTransform().toString());
      SmartDashboard.putNumber("Target Distance", getTargetDistance());
      
    }

    public int getTargetID(){
      if(hasTargets){
      return(target.getFiducialId());
      }
      return(0);
    }

    public Transform3d getTargetTransform(){
      if(hasTargets){
      return(target.getBestCameraToTarget());
      }
      return(new Transform3d());
    }

    public double getTargetDistance(){
      if(hasTargets){
        // Parameters: Camera Height M, Target Height M, Camera Pitch R, Target Pitch R
        double range = PhotonUtils.calculateDistanceToTargetMeters(
          1, 2,
          Math.PI, Units.degreesToRadians(target.getPitch()));
          return(range);
      }
      return(0);
    }

    public double getTargetYaw(){return target.getYaw();}

    public double getTargetPitch(){return target.getPitch();}

    public double getTargetSkew(){return target.getSkew();}

    public double getTargetArea(){return target.getArea();}

}
