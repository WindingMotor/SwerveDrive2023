// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.util;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.elevator.ElevatorSolenoid;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.commands.grabber.intake.GrabberHold;
import frc.robot.commands.grabber.intake.GrabberReverse;
import frc.robot.commands.grabber.intake.GrabberSolenoid;
import frc.robot.commands.led.deprecated.SetLedPurple;
import frc.robot.commands.led.deprecated.SetLedYellow;
import frc.robot.commands.routines.scoring.ScoreBottom;
import frc.robot.commands.routines.scoring.ScoreTop;
import frc.robot.commands.swerve.SwerveGoTo;
import frc.robot.commands.swerve.SwerveRotate;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.LightStrip;

public class SystemsCheck extends SequentialCommandGroup {

  public SystemsCheck(SwerveSubsystem swerveSubsystem,ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, LightStrip lightStrip){

   addCommands(
    new ReportString("Piston DOWN"),
    new ElevatorSolenoid(elevatorSubsystem),
    new WaitCommand(1),
    new ReportString("Piston UP"),
    new ElevatorSolenoid(elevatorSubsystem),
    new ReportString("Elevator TOP"),
    new ScoreTop(elevatorSubsystem, grabberSubsystem),
    new ReportString("Grabber OPEN"),
    new GrabberSolenoid(grabberSubsystem),
    new ReportString("Grabber HOLD"),
    new GrabberHold(grabberSubsystem),
    new WaitCommand(1),
    new ReportString("Grabber FORWARD"),
    new GrabberForward(grabberSubsystem),
    new WaitCommand(1),
    new ReportString("Grabber REVERSE"),
    new GrabberReverse(grabberSubsystem),
    new WaitCommand(1),
    new ReportString("Grabber CLOSE"),
    new GrabberSolenoid(grabberSubsystem),
    new WaitCommand(1),
    new ReportString("Elevator LOW"),
    new ScoreBottom(elevatorSubsystem, grabberSubsystem),
    new WaitCommand(1),
    new ReportString("Elevator ZERO"),
    new ScoreTop(elevatorSubsystem, grabberSubsystem),
    new ReportString("Swerve Y-AXIS"),
    new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(), 0, 1, 0, true, new Pose2d(), 0.1),
    new ReportString("Swerve X-AXIS"),
    new SwerveGoTo(swerveSubsystem, () -> swerveSubsystem.getHeading(), 1, 0, 0, true, new Pose2d(), 0.1),
    new ReportString("Swerve ROTATE"),
    new SwerveRotate(swerveSubsystem, 360.0),
    new SetLedPurple(lightStrip),
    new WaitCommand(1),
    new SetLedYellow(lightStrip)
   );

  }

}
