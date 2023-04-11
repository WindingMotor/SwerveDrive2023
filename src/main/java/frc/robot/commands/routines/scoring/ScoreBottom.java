// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.routines.scoring;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.grabber.angle.GrabberDegrees;

import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ScoreBottom extends SequentialCommandGroup{
    
    public ScoreBottom(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

        addCommands(
        // Set grabber angle to horizontal
        new GrabberDegrees(grabberSubsystem, 100),
        // Turn on intake
        // Move the elevator UP to platform height
        new ElevatorSetpoint(elevatorSubsystem, -0.9 /* Height of platform in meters */)
        );
    }
}