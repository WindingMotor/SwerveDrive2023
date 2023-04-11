// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.routines.scoring;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.grabber.angle.GrabberDegrees;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ScoreTop extends SequentialCommandGroup{
    
    public ScoreTop(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

        addCommands(
        // Set grabber angle to horizontal
        new GrabberDegrees(grabberSubsystem, 105),  // CHANGED TO 105 instead of 95 -> 4/7/2023
        // Turn on intake
        // Move the elevator UP to platform height
        new ElevatorSetpoint(elevatorSubsystem, -1.41 /* Height of platform in meters */)
        );

    }
}
