// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.routines.two;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.commands.elevator.ElevatorSolenoid;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.commands.grabber.intake.GrabberHold;
import frc.robot.commands.grabber.intake.GrabberReverse;
import frc.robot.commands.grabber.intake.GrabberSolenoid;
import frc.robot.commands.led.deprecated.SetLedBlue;
import frc.robot.commands.led.deprecated.SetLedGreen;
import frc.robot.commands.led.deprecated.SetLedPurple;
import frc.robot.commands.led.deprecated.SetLedRed;
import frc.robot.commands.led.deprecated.SetLedYellow;
import frc.robot.commands.routines.scoring.ScoreTop;
import frc.robot.commands.swerve.SwerveGoTo;
import frc.robot.commands.swerve.SwerveMove;
import frc.robot.commands.swerve.SwerveMoveRotate;
import frc.robot.commands.swerve.SwerveRotate;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.commands.util.ResetOdometryInverse;
import frc.robot.commands.util.ResetYaw;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LightStrip;

// Run multiple commands in a routine
public class TwoPieceWithLessPaths extends SequentialCommandGroup{

    // Routine command constructor
    public TwoPieceWithLessPaths(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController){

        addCommands(
        //new ResetOdometry(swerveSubsystem, new Pose2d()),
        //new ResetYaw(swerveSubsystem),

 
        //new ConeHigh(swerveSubsystem, elevatorSubsystem, grabberSubsystem, xController, yController, ppthetaController, ledStrip),
      
        new GrabberHold(grabberSubsystem), // reverse grabber for hold
        new ScoreTop(elevatorSubsystem, grabberSubsystem), // raise elevator
        new WaitCommand(0.5), // wait
        new ElevatorSolenoid(elevatorSubsystem), // bring down elevator
        new WaitCommand(1), // wait
        //new GrabberReverse(grabberSubsystem), // reverse grabber motor
        new GrabberSolenoid(grabberSubsystem), // open grabber up
        //new WaitCommand(0.5), //
        //new ElevatorSolenoid(elevatorSubsystem),
        //new SwerveGoTo(swerveSubsystem, ()->swerveSubsystem.getHeading(), 0, 0.5, 0, true, new Pose2d()),
        //new ElevatorZero(elevatorSubsystem, grabberSubsystem), // zero elevator
        new ParallelCommandGroup(
        new GrabberForward(grabberSubsystem), // Run grabber inwards

        new SequentialCommandGroup(
            new WaitCommand(0.2),
            new ElevatorZero(elevatorSubsystem, grabberSubsystem)),
        
        //new GrabberSolenoid(grabberSubsystem), // Open grabber
      

        // Move backwards and spin around
        //new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0,5.42,180.0, true, new Pose2d()),
        new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),0.2, 4.5, 160.0, false, null,0.1)
        ),

        new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),0.4, 5.35, 160, false, null,0.1),
        new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),0.2, 4.5, 0, false, null,0.1),

        // Grab game peice
        new ElevatorSolenoid(elevatorSubsystem),
        //new GrabberHold(grabberSubsystem),
       
        // Move forwards and rotate towards grid
        // new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(),0,-5.4,0, false, new Pose2d()),
        new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),.5, 0.1, 20, false, null,0.1),
      

        // Move sideways infront of high cube
        // new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(),1.0,0,180.0, false, new Pose2d())
        //new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),1.25, 0.1, 0.0, false, null),

        
        // Make sure angle is correct before scoring
        //new SwerveRotate(swerveSubsystem, 0),

       
        new GrabberHold(grabberSubsystem), // reverse grabber for hold
        new ScoreTop(elevatorSubsystem, grabberSubsystem), // raise elevator
        new WaitCommand(0.8), // wait
        new ElevatorSolenoid(elevatorSubsystem), // bring down elevator
        new WaitCommand(1), // wait
        new GrabberReverse(grabberSubsystem), // reverse grabber motor - only for cube ejecting
        new GrabberSolenoid(grabberSubsystem), // open grabber up
        new WaitCommand(0.5), // wait
        new ElevatorSolenoid(elevatorSubsystem), // bring up elevator
        new WaitCommand(1), // wait
        new ElevatorZero(elevatorSubsystem, grabberSubsystem) // zero elevator

        //new ElevatorSolenoid(elevatorSubsystem)


        //new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0.2,-5.4,0, false, new Pose2d()),

        // Move to grab the game peice
        //new SwerveMoveRotate(swerveSubsystem,() -> swerveSubsystem.getHeading(), 0.2,5.4,180.0, false),

        //new ResetYaw(swerveSubsystem), // reset gyro yaw
        //new ResetOdometryInverse(swerveSubsystem) // reset odometry

        );
    }
}