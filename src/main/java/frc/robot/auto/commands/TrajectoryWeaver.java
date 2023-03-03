// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot.auto.commands;
import java.util.function.Consumer;
import com.pathplanner.lib.PathPlannerTrajectory;
import com.pathplanner.lib.commands.PPSwerveControllerCommand;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.commands.util.ResetOdometry;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants.DriveConstants;

// Runs a given pp-trajectory as a command 
public class TrajectoryWeaver extends SequentialCommandGroup{
    
    Consumer<ChassisSpeeds> chassisSpeeds;

    // Constructor that obtains required values
    public TrajectoryWeaver(SwerveSubsystem swerveSubsystem,PIDController xController,
    PIDController yController,  PIDController ppthetaController,
    PathPlannerTrajectory pptrajectory, Boolean isFirstPath, Boolean autoBack){
      
        // Tell theta PID controller that its a circle
        ppthetaController.enableContinuousInput(-180, 180);

        // Check if first path
        if(isFirstPath){
           // Reset robot odometry before movement 
          addCommands(new ResetOdometry(swerveSubsystem, pptrajectory.getInitialHolonomicPose()));
        }else if(autoBack){
            //addCommands(new ResetOdometry(swerveSubsystem, pptrajectory.getInitialHolonomicPose().plus(new Transform2d(new Pose2d(), new Pose2d(0.0,0.0, new Rotation2d().fromDegrees(90))))));
        }

        addCommands(
            // Commands to run sequentially
            
            new SequentialCommandGroup(

              //                                              .......
              //       Yay ASKI art                          ...........
              //                                             <|><|><|>
              // Going insane with PPSwerveControllerCommand (0) o (0)
              // Fixed it... Java was being weird
            
              // Use Path Planner to move the swerve modules by letting it call setModuleStates
              new PPSwerveControllerCommand(pptrajectory, swerveSubsystem::getPose, DriveConstants.kDriveKinematics, xController, yController, ppthetaController, swerveSubsystem::setModuleStates, swerveSubsystem),
              
              // Tell driver station that command is running
             // new ReportWarning("Trajectory weaver: " + pptrajectory.toString(), true),
              // Stop all module movement
              new InstantCommand(() -> swerveSubsystem.stopModules())

            )
        ); 
        
        
        
        

    }

    public TrajectoryWeaver(SwerveSubsystem swerveSubsystem, PIDController xController, PIDController yController,
        PIDController ppThetaController, Trajectory trajectory, TrajectoryConfig trajectoryConfig, boolean b) {
    }
}