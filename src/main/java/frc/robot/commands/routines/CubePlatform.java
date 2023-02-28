// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.routines;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.grabber.GrabberDegrees;
import frc.robot.commands.grabber.GrabberIntake;
import frc.robot.commands.grabber.GrabberSolenoid;
import frc.robot.commands.util.ButtonFalse;
import frc.robot.subsystems.ButtonSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class CubePlatform extends SequentialCommandGroup{
    
    public CubePlatform(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, XboxController xbox){

        // If grabber is CLOSED open it for cube pickup
        boolean grabberOpen = grabberSubsystem.isGrabberOpen();
        if(!grabberOpen){
        addCommands(new GrabberSolenoid(grabberSubsystem));
        }
        
        addCommands(
        // Set grabber angle to horizontal
        new GrabberDegrees(grabberSubsystem, 38),
        // Turn on intake
        new GrabberIntake(grabberSubsystem),
        // Move the elevator UP to platform height
        new ElevatorSetpoint(elevatorSubsystem, -0.9 /* Height of platform in meters */),
        new ButtonFalse(xbox)
        );
    }
}