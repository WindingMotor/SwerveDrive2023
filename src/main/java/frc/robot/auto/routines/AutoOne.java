// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.auto.routines;
//import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.commands.util.ReportString;
import frc.robot.subsystems.SwerveSubsystem;



// Run multiple commands in a routine
public class AutoOne extends SequentialCommandGroup{

    // PPTrajectory and event map
    private final PathPlannerTrajectory pathOne = PathPlanner.loadPath("forward1M", new PathConstraints(4, 2)); 
    // private final HashMap<String, Command> eventMap = new HashMap<>();

    // Routine command constructor
    public AutoOne(SwerveSubsystem swerveSubsystem,PIDController xController,
    PIDController yController,  PIDController ppthetaController){

        // Add commands to event map markers
        // eventMap.put("marker1", new PrintCommand("TRAJ1: Passed Marker 1"));

        // Add commands to run
        addCommands(
        // Print out auto start
        new ParallelRaceGroup( new ReportString("AutoOne: Started"), new WaitCommand(0.1)),
        new TrajectoryWeaver(swerveSubsystem, xController, yController, ppthetaController, pathOne, true),
        new ParallelRaceGroup( new ReportString("AutoOne: Ended"), new WaitCommand(0.1))
        );
    }






    





}