// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.routines.scoring;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.grabber.angle.GrabberDegrees;
import frc.robot.commands.grabber.intake.GrabberReverseFast;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;

public class ObjectLaunch extends SequentialCommandGroup {

  public ObjectLaunch(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

   addCommands(new ElevatorSetpoint(elevatorSubsystem, -0.5),
   new GrabberDegrees(grabberSubsystem, 80),
   new GrabberReverseFast(grabberSubsystem));

  }

}