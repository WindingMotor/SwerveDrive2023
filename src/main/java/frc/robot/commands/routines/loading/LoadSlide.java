// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.routines.loading;
import java.util.function.Supplier;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.grabber.angle.GrabberDegrees;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.commands.grabber.intake.GrabberSolenoid;
import frc.robot.commands.swerve.SwerveAutoAlign;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Leds;

public class LoadSlide extends SequentialCommandGroup{
    
    public LoadSlide( ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

        // If the grabber is OPEN close it for clone pickup
        boolean grabberOpen = grabberSubsystem.isGrabberOpen();
        if(grabberOpen){
        addCommands(new GrabberSolenoid(grabberSubsystem));
        }
    
        addCommands(
        // Set grabber angle to horizontal
        new GrabberDegrees(grabberSubsystem, 110), // CHANGED TO 38 instead of 38 -> 3:00 
        // Turn on intake
        new GrabberForward(grabberSubsystem),
        // Move the elevator UP to platform height
        new ElevatorSetpoint(elevatorSubsystem, -0.18 /* Height of platform in meters */) // -1.28 to 1.3 - 0.7in differance
        );

    }
}