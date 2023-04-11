
// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.manuals;
import java.util.List;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.DriveConstants;

public class Backwards {

    // Create trajectory settings
    private static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(5, 5).setKinematics(DriveConstants.kDriveKinematics);
    private static double distance = 2.35;

//--------------------------------T-R-A-J-E-C-T-O-R-Y---S-T-A-R-T------------------------------//

    private static Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
    /* Starting point*/ new Pose2d(0,0,Rotation2d.fromDegrees(0)), List.of(
    /* Middle point*/ new Translation2d(0,distance/2)),
    /* Ending point*/new Pose2d(0,distance, Rotation2d.fromDegrees(-180)), trajectoryConfig);

//--------------------------------T-R-A-J-E-C-T-O-R-Y---E-N-D----------------------------------//

    public static void setDistance(double d){
        distance = d;
    }

    public static Trajectory getTrajectory(){
        return(trajectory);
    }
    
    public static TrajectoryConfig getTrajectoryConfig(){
        return(trajectoryConfig);
    }

}
