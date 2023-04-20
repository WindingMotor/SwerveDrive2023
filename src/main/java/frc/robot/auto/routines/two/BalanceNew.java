// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.routines.two;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.auto.routines.util.AutoBalance;
import frc.robot.commands.elevator.ElevatorSolenoid;
import frc.robot.commands.elevator.ElevatorZero;
import frc.robot.commands.grabber.intake.GrabberForward;
import frc.robot.commands.grabber.intake.GrabberHold;
import frc.robot.commands.grabber.intake.GrabberReverse;
import frc.robot.commands.grabber.intake.GrabberSolenoid;
import frc.robot.commands.led.deprecated.SetLedBlue;
import frc.robot.commands.led.deprecated.SetLedGreen;
import frc.robot.commands.led.deprecated.SetLedPurple;
import frc.robot.commands.led.deprecated.SetLedRed;
import frc.robot.commands.led.deprecated.SetLedYellow;
import frc.robot.commands.routines.scoring.ScoreTop;
import frc.robot.commands.swerve.SwerveGoTo;
import frc.robot.commands.swerve.SwerveGoToP;
import frc.robot.commands.swerve.SwerveMove;
import frc.robot.commands.swerve.SwerveMoveRotate;
import frc.robot.commands.swerve.SwerveRotate;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.commands.util.ResetOdometryInverse;
import frc.robot.commands.util.ResetYaw;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Leds;
import frc.robot.util.LightStrip;

// Run multiple commands in a routine
public class BalanceNew extends SequentialCommandGroup{

    // Routine command constructor
    public BalanceNew(SwerveSubsystem swerveSubsystem, ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, PIDController xController,
    PIDController yController,  PIDController ppthetaController, Leds leds){

        addCommands(

     
        new GrabberHold(grabberSubsystem), // Set grabber to hold mode
        new ScoreTop(elevatorSubsystem, grabberSubsystem), // Raise the elevator to high
        new WaitCommand(0.8), // Wait for elevator
        new ElevatorSolenoid(elevatorSubsystem), // Bring down pistons
        new WaitCommand(1), // Wait for pistons
        new GrabberSolenoid(grabberSubsystem), // Open the grabber
        new WaitCommand(0.5), // Wait for drop
        new ElevatorSolenoid(elevatorSubsystem), // Bring up pistons
        new WaitCommand(1), // Wait for pistons
        new ElevatorZero(elevatorSubsystem, grabberSubsystem), // Bring elevator down
        //new WaitCommand(0.8), // Wait for elevator



        // Move backwards and spin around - pick up game peice
        new SwerveGoToP(swerveSubsystem, () -> swerveSubsystem.getHeading(),0, 6.0, 0.0, true, new Pose2d(), 0.25, 2.0, 0.75),
        new WaitCommand(0.8),
        new SwerveGoToP(swerveSubsystem, () -> swerveSubsystem.getHeading(),0.4, 3, 0.0, false, null,0.1, 5, 0.5),
        new WaitCommand(1),
        new AutoBalance(swerveSubsystem, leds)

        //  new SwerveGoToP(swerveSubsystem, () -> swerveSubsystem.getHeading(),0.2, 4.5, 0, false, null,0.1, 2.0)

        );
    }
}
