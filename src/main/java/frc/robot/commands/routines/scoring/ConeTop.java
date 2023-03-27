// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.routines.scoring;
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

public class ConeTop extends SequentialCommandGroup{
    
    public ConeTop(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

        addCommands(
        // Set grabber angle to horizontal
        new GrabberDegrees(grabberSubsystem, 110),  // CHANGED TO 110 instead of 105 -> 9:00
        // Turn on intake
        // Move the elevator UP to platform height
        new ElevatorSetpoint(elevatorSubsystem, -1.4 /* Height of platform in meters */)
        );

    }
}
