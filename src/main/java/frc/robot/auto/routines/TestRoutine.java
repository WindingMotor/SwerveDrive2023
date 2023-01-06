// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.auto.routines;
import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.commands.ReportWarning;
import frc.robot.subsystems.SwerveSubsystem;

// Runs a given pp-trajectory as a command 
public class TestRoutine extends SequentialCommandGroup{



    
    // Routine command constructor
    public TestRoutine(SwerveSubsystem swerveSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController){

        // Create event map for this routine
        HashMap<String, Command> eventMap = new HashMap<>();

        // Add commands to markers
        eventMap.put("marker1", new PrintCommand("TRAJ1: Passed Marker 1"));

        // Import the paths to use
        PathPlannerTrajectory pathOne = PathPlanner.loadPath("pathOne", new PathConstraints(0.5, 0.5) /* velocity and acceleration */ ); 
        PathPlannerTrajectory pathTwo = PathPlanner.loadPath("pathTwo", new PathConstraints(0.5, 0.5) /* velocity and acceleration */ ); 

        // Add commands to routine
        addCommands(
        new ReportWarning("Running TRAJ1"),
        new TrajectoryWeaver(swerveSubsystem, xController, yController, ppthetaController, pathOne, eventMap, true),
        new ReportWarning("Running TRAJ2"),
        new TrajectoryWeaver(swerveSubsystem, xController, yController, ppthetaController, pathTwo, eventMap, false)
        );

    }






    





}