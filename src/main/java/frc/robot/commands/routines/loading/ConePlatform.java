// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.routines.loading;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.grabber.GrabberSolenoid;
import frc.robot.commands.grabber.angle.GrabberDegrees;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.commands.util.ButtonFalse;
import frc.robot.subsystems.ButtonSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ConePlatform extends SequentialCommandGroup{
    
    public ConePlatform(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

        // If the grabber is OPEN close it for clone pickup
        boolean grabberOpen = grabberSubsystem.isGrabberOpen();
        if(grabberOpen){
        addCommands(new GrabberSolenoid(grabberSubsystem));
        }
    
        addCommands(
        // Set grabber angle to horizontal
        new GrabberDegrees(grabberSubsystem, 38), // CHANGED TO 38 instead of 20 -> 9:00
        // Turn on intake
        new GrabberForward(grabberSubsystem),
        // Move the elevator UP to platform height
        new ElevatorSetpoint(elevatorSubsystem, -1.28 /* Height of platform in meters */)
        );

    }
}