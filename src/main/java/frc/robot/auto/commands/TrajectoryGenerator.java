// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.auto.commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
/* 
import java.util.HashMap;
import java.util.function.Consumer;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.commands.ReportString;
import frc.robot.subsystems.SwerveSubsystem;
*/


// Generate a command that goes to a specified position on the field
public class TrajectoryGenerator extends SequentialCommandGroup{

    /* 
    public TrajectoryGenerator(SwerveSubsystem swerveSubsystem,PIDController xController,
    PIDController yController,  PIDController ppthetaController,
    double x, double, y, double radian){

        PathPlannerTrajectory generatedTraj = PathPlanner.generatePath(
        AutoConstants.generatedPathConstraints, 
        new PathPoint(new Translation2d(x, y), Rotation2d.fromDegrees(0)));

        // Add commands to event map markers
        // eventMap.put("marker1", new PrintCommand("TRAJ1: Passed Marker 1"));

        // Add commands to run
        addCommands(
        new InstantCommand(swerveSubsystem::stopModules),
        // Print out auto start
        new ParallelRaceGroup( new ReportString("AutoOne: Started"), new WaitCommand(0.1)),
        new TrajectoryWeaver(swerveSubsystem, xController, yController, ppthetaController, generatedTraj, false),
        new ParallelRaceGroup( new ReportString("AutoOne: Ended"), new WaitCommand(0.1))
        );
    }

*/




    





}