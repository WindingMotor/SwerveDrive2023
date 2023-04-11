
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

public class Forward2M {

    // Create trajectory settings
    private static TrajectoryConfig trajectoryConfig = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond, AutoConstants.kMaxAccelerationMetersPerSecondSquared).setKinematics(DriveConstants.kDriveKinematics);

//--------------------------------T-R-A-J-E-C-T-O-R-Y---S-T-A-R-T------------------------------//

    private static Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
     /* Starting point*/ new Pose2d(0
     ,0,Rotation2d.fromDegrees(45)), List.of(
    // Interior points
    //new Translation2d(1,0),
    new Translation2d(0,0.25)),
    // Ending point
    new Pose2d(0,0.5, Rotation2d.fromDegrees(85)), trajectoryConfig);

//--------------------------------T-R-A-J-E-C-T-O-R-Y---E-N-D----------------------------------//

    public static Trajectory getTrajectory(){
        return(trajectory);
    }
    
    public static TrajectoryConfig getTrajectoryConfig(){
        return(trajectoryConfig);
    }

}
