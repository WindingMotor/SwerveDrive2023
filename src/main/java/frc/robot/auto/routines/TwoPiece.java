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
import frc.robot.commands.led.SetLedBlue;
import frc.robot.commands.led.SetLedGreen;
import frc.robot.commands.led.SetLedPurple;
import frc.robot.commands.led.SetLedRed;
import frc.robot.commands.led.SetLedYellow;
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

    // Routine command constructor
    public TwoPiece(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController, LightStrip ledStrip){

        addCommands(

        new SetLedYellow(ledStrip), // LED Yellow - Cone
        new GrabberHold(grabberSubsystem), // reverse grabber for hold
        new ScoreTop(elevatorSubsystem, grabberSubsystem), // raise elevator
        new WaitCommand(0.8), // wait
        new ElevatorSolenoid(elevatorSubsystem), // bring down elevator
        new WaitCommand(1), // wait
        //new GrabberReverse(grabberSubsystem), // reverse grabber motor - only for cube ejecting
        new GrabberSolenoid(grabberSubsystem), // open grabber up
        new WaitCommand(0.5), // wait
        new ElevatorSolenoid(elevatorSubsystem), // bring up elevator
        new WaitCommand(1), // wait
        new ElevatorZero(elevatorSubsystem, grabberSubsystem), // zero elevator
        new WaitCommand(1),
        new ElevatorSolenoid(elevatorSubsystem), 

        new GrabberForward(grabberSubsystem), // Run grabber inwards
        new GrabberSolenoid(grabberSubsystem), // Open grabber
        new SetLedRed(ledStrip), // LED Red - Reverse

        // Move backwards and spin around
        new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0,5.42,180.0, true, new Pose2d()),

        // Grab game peice
        new WaitCommand(0.2),
        new GrabberSolenoid(grabberSubsystem),
        new ElevatorSolenoid(elevatorSubsystem),
        new SetLedGreen(ledStrip), // LED Green - Forward

        // Move forwards and rotate twoards grid
        new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(),-0.05,-5.2,0, true, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180))),
        new WaitCommand(0.5),
        new SetLedPurple(ledStrip), // LED Purple - Cube

        // Move sideways infront of cube area
        new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(),-1.0,-5.2,0, false, new Pose2d(new Translation2d(), Rotation2d.fromDegrees(180)))

        //new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0.2,-5.4,0, false, new Pose2d()),

        // Move to grab the game peice
        //new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0.2,5.4,180.0, false),

        //new ResetYaw(swerveSubsystem), // reset gyro yaw
        //new ResetOdometryInverse(swerveSubsystem) // reset odometry

        );
    }
}