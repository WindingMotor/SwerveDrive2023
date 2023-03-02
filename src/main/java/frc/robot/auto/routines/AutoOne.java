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
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.routines.ConeBottom;
import frc.robot.commands.util.ReportString;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



// Run multiple commands in a routine
public class AutoOne extends SequentialCommandGroup{

    // PPTrajectory and event map
    private final PathPlannerTrajectory back = PathPlanner.loadPath("backwards0.5M", new PathConstraints(4, 2)); 
    // private final HashMap<String, Command> eventMap = new HashMap<>();

    // Routine command constructor
    public AutoOne(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController){

        // Add commands to event map markers
        // eventMap.put("marker1", new PrintCommand("TRAJ1: Passed Marker 1"));

        // Add commands to run
        addCommands(

 // new ConeTop(elevatorSubsystem, grabberSubsystem),
       // new WaitCommand(2.5),
      //  new ElevatorSolenoid(elevatorSubsystem),
     //   new WaitCommand(2),
      //  new GrabberSolenoid(grabberSubsystem),
      //  new WaitCommand(1),
      //  new ElevatorSolenoid(elevatorSubsystem),
      //  new WaitCommand(1.5),
        new TrajectoryWeaver(swerveSubsystem, xController, yController, ppthetaController, back, false, true)
     //   new ElevatorZero(elevatorSubsystem, grabberSubsystem),
     //   new WaitCommand(1),
     //   new GrabberSolenoid(grabberSubsystem)
        //new WaitCommand(2),
        //new ResetOdometry(swerveSubsystem, new Pose2d(0.0,0.0,new Rotation2d().fromDegrees(0)))




        );
    }






    





}