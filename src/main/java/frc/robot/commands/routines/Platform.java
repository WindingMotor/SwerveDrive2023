// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.commands.routines;
import java.sql.Driver;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.subsystems.ButtonSubsystem;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.SwerveSubsystem;

public class Platform extends SequentialCommandGroup {

  public Platform(ElevatorSubsystem elevatorSubsystem, GrabberSubsystem grabberSubsystem, XboxController xbox){


   // addCommands(new ConePlatform(elevatorSubsystem, grabberSubsystem, xbox));

    /* 
    if(xbox.getRawButton(6)){
      DriverStation.reportWarning("CONE CONE CONE", true);
      addCommands(new ConePlatform(elevatorSubsystem, grabberSubsystem, xbox));

    }
    // CUBE
    else if(xbox.getRawButton(5)){
      addCommands(new CubePlatform(elevatorSubsystem, grabberSubsystem, xbox));
    }
*/

  }

}