// FRC2106 Junkyard Dogs - Continuity Base Code - www.team2106.org

package frc.robot.auto.commands;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.commands.util.ResetYaw;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.DriveConstants;

// Runs a given trajectory as a command 
public class TrajectoryRunner extends SequentialCommandGroup{
    
    // Constructor that obtains required values
    public TrajectoryRunner(SwerveSubsystem swerveSubsystem, PIDController xController,
    PIDController yController,  ProfiledPIDController thetaController,
    Trajectory trajectory, TrajectoryConfig trajectoryConfig){

        // Tell theta PID controller that its a circle
        thetaController.enableContinuousInput(-180, 180);

         // Create controller command, this outputs module states for the trajectory given
       // SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem);

        // Create report warning command, prints running trajectory to driver station
       // ReportWarning sendData = new ReportWarning("Trajectory runner: " + trajectory.toString(), true);

        addCommands(
            // Commands to run sequentially
            new SequentialCommandGroup(
                new ResetOdometry(swerveSubsystem, trajectory.getInitialPose()),  // Reset robot odometry before movement 
                new ResetYaw(swerveSubsystem),
                new SwerveControllerCommand(trajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, thetaController, swerveSubsystem::setModuleStates, swerveSubsystem), // Move robot with trajectory and module states
                new InstantCommand(() -> swerveSubsystem.stopModules()) // Stop all modules
            )
        ); 

    }
}














/* He be runnin

                        ,////,
                        /// 6|
                        //  _|
                       _/_,-'
                  _.-/'/   \   ,/;,
               ,-' /'  \_   \ / _/
               `\ /     _/\  ` /
                 |     /,  `\_/
                 |     \'
     /\_        /`      /\
   /' /_``--.__/\  `,. /  \
  |_/`  `-._     `\/  `\   `.
            `-.__/'     `\   |
                          `\  \
                            `\ \
                              \_\__
                               \___)
    
*/