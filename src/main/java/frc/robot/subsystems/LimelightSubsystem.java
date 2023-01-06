// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.util.Constants.VisionConstants;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class LimelightSubsystem extends SubsystemBase{
    
    // Create private instance variables  
    NetworkTable table;
    private NetworkTableEntry tx;
    private NetworkTableEntry ty;
    private NetworkTableEntry ta;

    // Caculation variables
    private double distance;
    private double[] all = new double[2];
    private double x;
    private double y;
    private double a;

    // Subsystem Constructor
    public LimelightSubsystem(){

    // Get limelight from network tables
    NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");

    // Set variables from limelight network tables
    // tx and ty are offsets to target in degrees and ta is total target area
    NetworkTableEntry tx = table.getEntry("tx");
    NetworkTableEntry ty = table.getEntry("ty");
    NetworkTableEntry ta = table.getEntry("ta");
}

    // Method to update main caculation variables
    private void update(){
        x = tx.getDouble(0);
        y = ty.getDouble(0);
        a = ta.getDouble(0);
        all[0] = x;
        all[1] = y;
        all[2] = a;
        distance = getDistance(true);
    }

    // Change camera pipelines of limelight
    public void setView(int v){
        if( v == 0){
            // Set limelight pipeline view to 0
            table.getEntry("pipeline").setNumber(0);
        } else if(v == 1){
            // Set limelight pipeline view to 1
            table.getEntry("pipeline").setNumber(1);
        }
    }

    // Get methods
    public double getX(){return x;}

    public double getY(){return y;}

    public double getA(){return a;}

    // Return an array of all get method values
    public double[] getAll(){return all;}

    // Caculate distance from camera to target
    public double getDistance(boolean direct){
        // Get value directly or caculate it
        if(direct == true){
            return VisionConstants.deltaHeight/(
            Math.tan(
            Math.toRadians(
            VisionConstants.cameraAngle + y
            )));
            
        }
        else{return distance;}
    }

    // Update vision variables once per scheduler run
    @Override
    public void periodic(){update();}
}
