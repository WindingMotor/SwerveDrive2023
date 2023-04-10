// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.auto.routines;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.commands.elevator.ElevatorSolenoid;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.commands.grabber.intake.GrabberHold;
import frc.robot.commands.grabber.intake.GrabberReverse;
import frc.robot.commands.grabber.intake.GrabberSolenoid;
import frc.robot.commands.led.SetLedPurple;
import frc.robot.commands.led.SetLedRed;
import frc.robot.commands.routines.scoring.ScoreTop;
import frc.robot.commands.swerve.SwerveMove;
import frc.robot.commands.swerve.SwerveMoveRotate;
import frc.robot.commands.util.ResetOdometryInverse;
import frc.robot.commands.util.ResetYaw;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LightStrip;



// Run multiple commands in a routine
public class TwoPiece extends SequentialCommandGroup{


    // DEPRECATED

    // PPTrajectory and event map
    private final PathPlannerTrajectory back = PathPlanner.loadPath("backwards0.5M", new PathConstraints(3.5, 4.5)); 
    // private final HashMap<String, Command> eventMap = new HashMap<>();

    // Routine command constructor
    public TwoPiece(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController, LightStrip ledStrip){

        // Add commands to event map markers
        // eventMap.put("marker1", new PrintCommand("TRAJ1: Passed Marker 1"));

        new Rotation2d();
        // Add commands to run
        addCommands(
        new SetLedRed(ledStrip),
        new GrabberHold(grabberSubsystem), // reverse grabber for hold
        new ScoreTop(elevatorSubsystem, grabberSubsystem), // raise elevator
        new WaitCommand(0.8), // wait
        new ElevatorSolenoid(elevatorSubsystem), // bring down elevator
        new WaitCommand(1), // wait
        //new GrabberReverse(grabberSubsystem), // reverse grabber motor
        new GrabberSolenoid(grabberSubsystem), // open grabber up
        new WaitCommand(0.5), // wait
        new ElevatorSolenoid(elevatorSubsystem), // bring up elevator
        new WaitCommand(1), // wait
        new ElevatorZero(elevatorSubsystem, grabberSubsystem), // zero elevator
        new WaitCommand(1),
        new ElevatorSolenoid(elevatorSubsystem), 

        // Run grabber inwards
         new GrabberForward(grabberSubsystem),

        // Open grabber
        new GrabberSolenoid(grabberSubsystem),

        // Move infront of the game peice 

        new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0,5.42,180.0, true, new Pose2d()),
        new WaitCommand(0.2),
        new GrabberSolenoid(grabberSubsystem),
        new ElevatorSolenoid(elevatorSubsystem),
        new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(),-0.05,-5.2,0, true, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)))

        //new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0.2,-5.4,0, false, new Pose2d()),

        // Move to grab the game peice
        //new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0.2,5.4,180.0, false),

        // Close the grabber

        //new ResetYaw(swerveSubsystem), // reset gyro yaw
        //new ResetOdometryInverse(swerveSubsystem) // reset odometry
        );

    }






    





}