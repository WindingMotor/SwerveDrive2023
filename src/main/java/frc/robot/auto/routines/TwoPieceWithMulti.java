// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.routines;
import java.util.ArrayList;
import java.util.List;

import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
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
import frc.robot.commands.led.SetLedBlue;
import frc.robot.commands.led.SetLedGreen;
import frc.robot.commands.led.SetLedPurple;
import frc.robot.commands.led.SetLedRed;
import frc.robot.commands.led.SetLedYellow;
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
import frc.robot.commands.swerve.SetPoint;
import frc.robot.commands.swerve.SwerveGoToMulti;

// Run multiple commands in a routine
public class TwoPieceWithMulti extends SequentialCommandGroup{

    // Routine command constructor
    public TwoPieceWithMulti(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController, LightStrip ledStrip){

    List<SetPoint> setpoints = new ArrayList<>();
    setpoints.add(new SetPoint(0, 4.5, 160.0, true, new Pose2d()));
    setpoints.add(new SetPoint(0.4, 5.35, 160, false, null));
    setpoints.add(new SetPoint(0.2, 4.5, 0, false, null));

        addCommands(

        new SetLedYellow(ledStrip), // Set led to yellow - Cone
        new GrabberHold(grabberSubsystem), // Set grabber to hold mode
        new ScoreTop(elevatorSubsystem, grabberSubsystem), // Raise the elevator to high
        new WaitCommand(0.8), // Wait for elevator
        new ElevatorSolenoid(elevatorSubsystem), // Bring down pistons
        new WaitCommand(1), // Wait for pistons
        new GrabberSolenoid(grabberSubsystem), // Open the grabber
        new GrabberForward(grabberSubsystem), // Run grabber inwards
        new ElevatorSolenoid(elevatorSubsystem), // Bring pistons up
        new WaitCommand(0.8), // Wait for elevator
        new ElevatorZero(elevatorSubsystem, grabberSubsystem), // Bring elevator down

        new ParallelCommandGroup(
            new SequentialCommandGroup(
                new WaitCommand(0.3),
                new ElevatorSolenoid(elevatorSubsystem)
            ),

            new SwerveGoToMulti(swerveSubsystem,()-> swerveSubsystem.getHeading(), setpoints,0.02)
        ),

        new ElevatorSolenoid(elevatorSubsystem), // Bring up pistons
        new GrabberHold(grabberSubsystem), // Set grabber to hold mode
        
        // Move forwards and rotate towards grid
        new SetLedGreen(ledStrip), // LED Green - Forward
        new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),0.5, 0.2, 0.0, false, null,0.08),

        // Move sideways infront of high cube
        new SetLedPurple(ledStrip), // LED Purple - Cube
        new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),1.25, 0.2, 0.0, false, null,0.08),

        
        // Make sure angle is correct before scoring
        //new SwerveRotate(swerveSubsystem, 0),

        new SetLedPurple(ledStrip), 
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

        );
    }
}
