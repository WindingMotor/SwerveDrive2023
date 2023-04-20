// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.routines.util;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.swerve.SwerveAutoBalance;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Leds;
import frc.robot.util.LightStrip;

// Run multiple commands in a routine
public class AutoBalance extends SequentialCommandGroup{

    private double waitTime = 0.2;

    // Routine command constructor
    public AutoBalance(SwerveSubsystem swerveSubsystem, Leds leds){
        
        addCommands(
        new SwerveAutoBalance(swerveSubsystem, () -> swerveSubsystem.getHeading(), leds),
        new WaitCommand(waitTime),

        new SwerveAutoBalance(swerveSubsystem, () -> swerveSubsystem.getHeading(), leds),
        new WaitCommand(waitTime),

        new SwerveAutoBalance(swerveSubsystem, () -> swerveSubsystem.getHeading(), leds),
        new WaitCommand(waitTime),

        new SwerveAutoBalance(swerveSubsystem, () -> swerveSubsystem.getHeading(), leds)

        );
    }
}
