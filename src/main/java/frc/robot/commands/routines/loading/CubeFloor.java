// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.routines.loading;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.grabber.GrabberSolenoid;
import frc.robot.commands.grabber.angle.GrabberDegrees;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class CubeFloor extends SequentialCommandGroup{
    
    public CubeFloor(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

        // If the grabber is CLOSED close it for clone pickup
        boolean grabberOpen = grabberSubsystem.isGrabberOpen();
        if(!grabberOpen){
        addCommands(new GrabberSolenoid(grabberSubsystem));
        }
    
        addCommands(
        // Set grabber angle to horizontal
        new GrabberDegrees(grabberSubsystem, 38),
        // Turn on intake
        new GrabberForward(grabberSubsystem),
        // Move the elevator DOWN to floor height
        new ElevatorSetpoint(elevatorSubsystem, 0.952 /* Height of platform in meters */)
        );
    }
}