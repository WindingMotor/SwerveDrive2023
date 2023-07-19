// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.routines.two;
import java.util.ArrayList;
import java.util.List;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorSolenoid;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.commands.grabber.intake.GrabberHold;
import frc.robot.commands.grabber.intake.GrabberReverse;
import frc.robot.commands.grabber.intake.GrabberSolenoid;
import frc.robot.commands.led.deprecated.SetLedGreen;
import frc.robot.commands.led.deprecated.SetLedPurple;
import frc.robot.commands.led.deprecated.SetLedYellow;
import frc.robot.commands.routines.scoring.ScoreTop;
import frc.robot.commands.swerve.SwerveGoTo;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LightStrip;
import frc.robot.commands.swerve.SetPoint;
import frc.robot.commands.swerve.SwerveGoToMultiP;

// Run multiple commands in a routine
public class ConeCubeHighBump extends SequentialCommandGroup{

    // Routine command constructor
    public ConeCubeHighBump(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController, boolean isRed){


    if(isRed){

        List<SetPoint> setpoints = new ArrayList<>();
    setpoints.add(new SetPoint(-0.6, 4.5, 200, true, new Pose2d()));
    setpoints.add(new SetPoint( /* */ -0.81, 5.6, 200, false, null));
    setpoints.add(new SetPoint(-0.5, 4.5, 0, false, null));

        addCommands(

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

            new SwerveGoToMultiP(swerveSubsystem,()-> swerveSubsystem.getHeading(), setpoints,0.1,3)
        ),

        new ElevatorSolenoid(elevatorSubsystem), // Bring up pistons
        new GrabberHold(grabberSubsystem), // Set grabber to hold mode
        
        // Move forwards and rotate towards grid
   
        new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),-0.5, 0.15, 0.0, false, null,0.1),

        // Move sideways infront of high cube

        new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),-0.75, 0.15, 0.0, false, null,0.1),

        
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

        );


    }else{

        List<SetPoint> setpoints = new ArrayList<>();
        setpoints.add(new SetPoint(0, 4.5, 165, true, new Pose2d()));
        setpoints.add(new SetPoint(0.24, 5.85, 165, false, null));
        setpoints.add(new SetPoint(0.2, 4.5, 0, false, null));
        //0.25, 6.18

            addCommands(
    
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
    
                new SwerveGoToMultiP(swerveSubsystem,()-> swerveSubsystem.getHeading(), setpoints,0.1,4)
            ),
    
            new ElevatorSolenoid(elevatorSubsystem), // Bring up pistons
            new GrabberHold(grabberSubsystem), // Set grabber to hold mode
            
            // Move forwards and rotate towards grid
          
            new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),0.5, 0.25, 0.0, false, null,0.1),
    
            // Move sideways infront of high cube
       
            new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(),0.95, 0.35, 0.0, false, null,0.1),
    
            
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
    
            );
    }
    }
}
