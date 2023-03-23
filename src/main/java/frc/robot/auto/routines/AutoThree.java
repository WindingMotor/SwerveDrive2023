// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.auto.routines;
//import java.util.HashMap;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.PathPoint;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
//import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.SetBackwardsValue;
import frc.robot.auto.commands.TrajectoryRunner;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.auto.manuals.Backwards;
import frc.robot.auto.manuals.TestTrajectory;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.elevator.ElevatorSolenoid;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.grabber.GrabberSolenoid;
import frc.robot.commands.routines.scoring.ConeBottom;
import frc.robot.commands.routines.scoring.ConeTop;
import frc.robot.commands.util.ReportString;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.commands.util.ResetOdometryInverse;
import frc.robot.commands.util.ResetYaw;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;



// Run multiple commands in a routine
public class AutoThree extends SequentialCommandGroup{

    // Routine command constructor
    public AutoThree(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  ProfiledPIDController thetaController){

        // Add commands to run
        addCommands(
        // Bring elevator to top
        new ConeTop(elevatorSubsystem, grabberSubsystem),
        new WaitCommand(2.5),
        // Change elevator angle
        new ElevatorSolenoid(elevatorSubsystem),
        new WaitCommand(2),
        // Open grabber
        new GrabberSolenoid(grabberSubsystem),
        new WaitCommand(1),
        // Change elevator angle
        new ElevatorSolenoid(elevatorSubsystem),
        new WaitCommand(1.5),
        // Bring elevator down
        new ElevatorZero(elevatorSubsystem, grabberSubsystem),
        new WaitCommand(1),
        // Set backwards value to 1 meter
       // new SetBackwardsValue(1.0),
        // Start driving onto charge station
        new TrajectoryRunner(swerveSubsystem, xController, yController, thetaController, Backwards.getTrajectory(), Backwards.getTrajectoryConfig()),
        new WaitCommand(2)

        );
    }
}