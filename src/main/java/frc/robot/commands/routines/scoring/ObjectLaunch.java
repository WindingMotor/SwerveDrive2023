// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.routines.scoring;
import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.elevator.ElevatorSetpoint;
import frc.robot.commands.grabber.angle.GrabberDegrees;
import frc.robot.commands.grabber.intake.GrabberReverseFast;
import frc.robot.subsystems.ButtonSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class ObjectLaunch extends SequentialCommandGroup {

  public ObjectLaunch(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem){

   addCommands(new ElevatorSetpoint(elevatorSubsystem, -0.5),
   new GrabberDegrees(grabberSubsystem, 80),
   new GrabberReverseFast(grabberSubsystem));

  }

}