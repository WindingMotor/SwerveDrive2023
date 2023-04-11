// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.routines;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



// Run multiple commands in a routine
public class AutoTwo extends SequentialCommandGroup{

    // PPTrajectory and event map
    //private final PathPlannerTrajectory back = PathPlanner.loadPath("backwardsOne", new PathConstraints(3.5, 4.5)); 


    PathPlannerTrajectory traj1 = PathPlanner.generatePath(
    new PathConstraints(5, 5), 
    new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)),
    new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0))
   // new PathPoint(new Translation2d(1.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)) 
    );



// More complex path with holonomic rotation. Non-zero starting velocity of 2 m/s. Max velocity of 4 m/s and max accel of 3 m/s^2
PathPlannerTrajectory traj3 = PathPlanner.generatePath(
    new PathConstraints(4, 3), 
    new PathPoint(new Translation2d(0.0, 0.0), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(0)), // position, heading(direction of travel), holonomic rotation, velocity override
    new PathPoint(new Translation2d(1.5, 1.5), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-90)) // position, heading(direction of travel), holonomic rotation
    //new PathPoint(new Translation2d(2.7, 2.7), Rotation2d.fromDegrees(0), Rotation2d.fromDegrees(-120)) // position, heading(direction of travel), holonomic rotation
);


    // private final HashMap<String, Command> eventMap = new HashMap<>();

    // Routine command constructor
    public AutoTwo(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController){

        // Add commands to event map markers
        // eventMap.put("marker1", new PrintCommand("TRAJ1: Passed Marker 1"));

        // Add commands to run
        addCommands(
        
        new TrajectoryWeaver(swerveSubsystem, xController, yController, ppthetaController, traj3, true, false)

        );

    }






    





}