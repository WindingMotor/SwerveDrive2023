// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.routines;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorMeters;
import frc.robot.commands.elevator.ElevatorMetersJoystick;
import frc.robot.commands.grabber.GrabberDegrees;
import frc.robot.commands.grabber.GrabberIntake;
import frc.robot.commands.grabber.GrabberSolenoid;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class CubePlatform extends SequentialCommandGroup{
    
    public CubePlatform(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

        // If grabber is CLOSED open it for cube pickup
        boolean grabberOpen = grabberSubsystem.isGrabberOpen();
        if(!grabberOpen){
        addCommands(new GrabberSolenoid(grabberSubsystem));
        }
        
        addCommands(
        // Set grabber angle to horizontal
        new GrabberDegrees(grabberSubsystem, 12.0),
        // Turn on intake
        new GrabberIntake(grabberSubsystem),
        // Move the elevator UP to platform height
        new ElevatorMeters(elevatorSubsystem, 0.952 /* Height of platform in meters */)
        );
    }
}