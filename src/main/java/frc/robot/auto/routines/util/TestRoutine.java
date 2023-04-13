// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.routines.util;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.subsystems.SwerveSubsystem;

// Run multiple commands in a routine
public class TestRoutine extends SequentialCommandGroup{

    // Routine command constructor
    public TestRoutine(SwerveSubsystem swerveSubsystem,PIDController xController,
    PIDController yController,  PIDController ppthetaController){

        // Create event map for this routine
        //HashMap<String, Command> eventMap = new HashMap<>();

        // Add commands to markers
       // eventMap.put("marker1", new PrintCommand("TRAJ1: Passed Marker 1"));

        // Import the paths to use
        PathPlannerTrajectory pathOne = PathPlanner.loadPath("backwardsOne", new PathConstraints(0.5, 0.5) /* velocity and acceleration */ ); 
        //PathPlannerTrajectory pathTwo = PathPlanner.loadPath("pathTwo", new PathConstraints(0.5, 0.5) /* velocity and acceleration */ ); 

        // Add commands to routine
        addCommands(

        //new ReportWarning("Running TRAJ1", true),
        new TrajectoryWeaver(swerveSubsystem, xController, yController, ppthetaController, pathOne, true, false)
       // new ReportWarning("Running TRAJ2", true),
        //new TrajectoryWeaver(swerveSubsystem, xController, yController, ppthetaController, pathTwo, eventMap, false)

        );

    }






    





}