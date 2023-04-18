// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.commands.led.deprecated.shows;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.led.deprecated.DisableLedRainbow;
import frc.robot.commands.led.deprecated.SetLedRGBEveryOther;
import frc.robot.util.LightStrip;

// Run multiple commands in a routine
public class LedLightShowOne extends SequentialCommandGroup{

    // Routine command constructor
    public LedLightShowOne(LightStrip ledStrip){

      addCommands(
        new DisableLedRainbow(ledStrip),
        new SetLedRGBEveryOther(ledStrip, 162, 0, 0, 255, 255, 255),
        new WaitCommand(0.3),
        new SetLedRGBEveryOther(ledStrip, 255, 255, 255, 162, 0, 0),
        new WaitCommand(0.3),

        new SetLedRGBEveryOther(ledStrip, 162, 0, 0, 255, 255, 255),
        new WaitCommand(0.3),
        new SetLedRGBEveryOther(ledStrip, 255, 255, 255, 162, 0, 0),
        new WaitCommand(0.3),

        new SetLedRGBEveryOther(ledStrip, 162, 0, 0, 255, 255, 255),
        new WaitCommand(0.3),
        new SetLedRGBEveryOther(ledStrip, 255, 255, 255, 162, 0, 0),
        new WaitCommand(0.3),

        new SetLedRGBEveryOther(ledStrip, 162, 0, 0, 255, 255, 255),
        new WaitCommand(0.3),
        new SetLedRGBEveryOther(ledStrip, 255, 255, 255, 162, 0, 0),
        new WaitCommand(0.3)
      );

    }
  }