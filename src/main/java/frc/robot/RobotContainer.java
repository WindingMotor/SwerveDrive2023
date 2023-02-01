// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import frc.robot.auto.commands.TrajectoryRunner;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.auto.manuals.Forward2M;
import frc.robot.auto.routines.AutoOne;
import frc.robot.auto.routines.TestRoutine;
import frc.robot.commands.GrabberClose;
import frc.robot.commands.GrabberOpen;
import frc.robot.commands.ResetPose;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.SwerveRotator;
import frc.robot.commands.SwerveThrottledJoystick;
import frc.robot.subsystems.GrabberSubsystem;
import frc.robot.subsystems.LimelightSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.Transmitter;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.IOConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;

// Ignore unused variable warnings
@SuppressWarnings("unused")

public class RobotContainer {

  // Converted to 2023 wpiblib

  //------------------------------------O-B-J-E-C-T-S-----------------------------------//

  // Create joysticks
  private final Joystick leftJoystick = new Joystick(IOConstants.kLeftJoystick);
  private final Joystick rightJoystick = new Joystick(IOConstants.kRightJoystick);

  // Create swerve subsystem
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem(rightJoystick);
  
  private final GrabberSubsystem grabberSubsystem =  new GrabberSubsystem();
  // Create Xbox controller
  private final XboxController xboxController = new XboxController(IOConstants.kXboxController);

  // Create PID controllers for trajectory tracking
  public final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  public final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  public final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);


  // Create a non profiled PID controller for path planner
  private final PIDController ppThetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

  // Create transmitter object
  private final Transmitter transmitter = new Transmitter(4);

  private final XboxController xbox = new XboxController(4);

  PathPlannerTrajectory pathOne = PathPlanner.loadPath("forward1M", new PathConstraints(0.25, 0.25)); 


  //------------------------------------C-O-N-S-T-R-U-C-T-O-R----------------------------//

  public RobotContainer(){

    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
    // Joystick Numbers 0 = LEFT : 1 = RIGHT
    // Joystick Axises: 0 = left/right : 1 = forward/backwards : 2 = dial
    // Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2

  //>-------------N-O-R-M-A-L----------------<//
   
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> rightJoystick.getRawAxis(0), // X-Axis
    () -> rightJoystick.getRawAxis(1), // Y-Axis
    () -> leftJoystick.getRawAxis(0), // R-Axis
    () -> trueFunct(),
    () -> swerveSubsystem.getHeading())); 

    /* 
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> xbox.getRawAxis(4), // X-Axis
    () -> xbox.getRawAxis(5), // Y-Axis
    () -> xbox.getRawAxis(0), // R-Axis
    () -> trueFunct(),
    () -> swerveSubsystem.getHeading())); 
    */

    /* DEBUG SETUP -> ZERO MOVEMENT
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> zeroFunct(), // X-Axis
    () -> rightJoystick.getRawAxis(2), // Y-Axis
    () -> zeroFunct(), // R-Axis
    () -> !leftJoystick.getRawButton(Constants.IOConstants.kFieldOrientedButton)));
    */
    
  //>--------------T-R-A-N-S-----------------//
    /* 
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> transmitter.getRoll(), // X-Axis
    () -> -transmitter.getPitch(), // Y-Axis
    () -> transmitter.getYaw(), // R-Axis
    () -> transmitter.getSwitchVeryRight(),
    () -> swerveSubsystem.getHeading() )); 
    */
  //>----------T-H-R-T-L----------<// // No clue if working...
    /*
    swerveSubsystem.setDefaultCommand(new SwerveThrottledJoystick(swerveSubsystem,
    () -> rightJoystick.getRawAxis(0), // X-Axis
    () -> rightJoystick.getRawAxis(1), // Y-Axis
    () -> leftJoystick.getRawAxis(0), // R-Axis
    () -> leftJoystick.getRawAxis(1), // T-Axis
    () -> !leftJoystick.getRawButton(Constants.IOConstants.kFieldOrientedButton))); // Field Oriented
    */

    // Run button binding method
    configureButtonBindings();
  }

  //------------------------------------D-E-B-U-G------------------------------------//

  private double zeroFunct(){return 0;}

  private boolean trueFunct(){return true;}

  //------------------------------------B-U-T-T-O-N-S------------------------------------//
 
  // Create buttons bindings
  private void configureButtonBindings(){

    /* 
    new JoystickButton(xbox, 7).onTrue(new GrabberOpen(limelightSubsystem));
   new JoystickButton(xbox, 8).onTrue(new GrabberClose(limelightSubsystem));
   new JoystickButton(xbox, 1).onTrue(new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, false));
    //new JoystickButton(leftJoystick, 0).onTrue() ->swerveSubsystem.zeroHeading()
    */

    new JoystickButton(rightJoystick, 1).onTrue(new GrabberOpen(grabberSubsystem));
    new JoystickButton(leftJoystick, 1).onTrue(new GrabberClose(grabberSubsystem));
    new JoystickButton(rightJoystick, 2).onTrue(new ResetPose(swerveSubsystem, new Pose2d()));
    //new JoystickButton(rightJoystick, 3).onTrue(new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, true));

  // DEPRECATED 2023
  //new JoystickButton(rightJoystick,Constants.IOConstants.kZeroHeadingButton).whenPressed(() -> swerveSubsystem.zeroHeading());
  // DEPRECATED 2023

    // Rotate robot 90* using swerve rotator
    //new JoystickButton(leftJoystick, Constants.IOConstants.kRotatorButton).whenPressed(new SwerveRotator(swerveSubsystem, () -> 0.1, swerveSubsystem.getHeading()));

  }

  //------------------------------------R-E-F-E-R-R-E-R-S------------------------------------//

    public void containerResetAllEncoders(){
      DriverStation.reportWarning("Running containerResetAllEncoders() in RobotContainer", true);
      swerveSubsystem.resetAllEncoders();
    }

  //------------------------------------A-U-T-O-N-O-M-O-U-S------------------------------------//
  

  private Command autoOne = new AutoOne(swerveSubsystem, xController, yController, ppThetaController);
  
  private Command forward = new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, true);

  // Return the command to run during auto
  public Command getAutonomousCommand(){

  // Name of command to run for selector
  String autoSelector = "forward2M";
  // Command to run
  Command autoCommand = null;

  //------------------------------------S-E-L-E-C-T-O-R------------------------------------//

  /* 
    // Selector if-statement
    if(autoSelector == "forward2M"){
      autoCommand = forward2M;
    }
    else if(autoSelector == "testRoutine"){
      autoCommand = testRoutine;
    }\
    */

    return forward;
  }
}