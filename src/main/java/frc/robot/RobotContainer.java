// FRC2106 Junkyard Dogs - Swerve Drive Base Code

package frc.robot;
import com.pathplanner.lib.PathConstraints;
import com.pathplanner.lib.PathPlanner;
import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.auto.commands.TrajectoryRunner;
import frc.robot.auto.commands.TrajectoryWeaver;
import frc.robot.auto.manuals.Forward2M;
import frc.robot.auto.routines.AutoOne;
import frc.robot.auto.routines.TestRoutine;
import frc.robot.commands.GrabberToggle;
import frc.robot.commands.ResetOdometry;
import frc.robot.commands.SwerveAlignBasic;
import frc.robot.commands.SwerveJoystick;
import frc.robot.commands.SwerveRotator;
import frc.robot.commands.SwerveThrottledJoystick;
import frc.robot.subsystems.ElevatorSubsystem;
import frc.robot.subsystems.SwerveSubsystem;
import frc.robot.subsystems.VisionSubsystem;
import frc.robot.util.Constants;
import frc.robot.util.Transmitter;
import frc.robot.util.Constants.AutoConstants;
import frc.robot.util.Constants.IOConstants;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
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
  private final SwerveSubsystem swerveSubsystem = new SwerveSubsystem();
  
  // Create lift subsystem
  private final ElevatorSubsystem grabberSubsystem = new ElevatorSubsystem();

  // Create vision subsystem
  private final VisionSubsystem visionSubsystem = new VisionSubsystem();

  // Create PID controllers for trajectory tracking
  public final PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
  public final PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
  public final ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, AutoConstants.kIThetaController, AutoConstants.kDThetaController, AutoConstants.kThetaControllerConstraints);

  // Create a non profiled PID controller for path planner
  private final PIDController ppThetaController = new PIDController(AutoConstants.kPThetaController, 0, 0);

  // Create transmitter object
  private final Transmitter transmitter = new Transmitter(4);

  // Create xbox controller
  private final XboxController xbox = new XboxController(4);

  // Create transmitter
  private final Joystick tx16s = new Joystick(4);


  //-------------------------------------------------------------------------------------//

 
  // Manual path 
  private final PathPlannerTrajectory pathOne = PathPlanner.loadPath("forward1M", new PathConstraints(0.25, 0.25)); 
  private Command autoForward = new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, true);

  // Routine
  private Command autoOne = new AutoOne(swerveSubsystem, xController, yController, ppThetaController);

  private SendableChooser<Command> autoChooser = new SendableChooser<>();


  //------------------------------------C-O-N-S-T-R-U-C-T-O-R----------------------------//

  public RobotContainer(){

    // Set swerve subsystem default command to swerve joystick with respective joystick inputs
    // Joystick Numbers 0 = LEFT : 1 = RIGHT
    // Joystick Axises: 0 = left/right : 1 = forward/backwards : 2 = dial
    // Transmitter Axises: 0 = roll : 1 = pitch : 2 = throttle : 3 = yaw : 4 = analog1 : 5 = analog2

  //>-------------N-O-R-M-A-L----------------<//
   
  /* 
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> rightJoystick.getRawAxis(0), // X-Axis
    () -> rightJoystick.getRawAxis(1), // Y-Axis
    () -> leftJoystick.getRawAxis(0), // R-Axis
    () -> trueFunct(),
    () -> swerveSubsystem.getHeading())); 
    */

    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> leftJoystick.getRawAxis(0), // X-Axis
    () -> leftJoystick.getRawAxis(1), // Y-Axis
    () -> rightJoystick.getRawAxis(0), // R-Axis
    () -> tx16s.getRawButton(0), // Field oriented -does nothing right now
    () -> swerveSubsystem.getHeading(), // Navx heading
    () -> leftJoystick.getRawButton(1))); // Flick offset button, should be toggle!

/* 
swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
() -> xbox.getRawAxis(4), // X-Axis
() -> xbox.getRawAxis(5), // Y-Axis
() -> xbox.getRawAxis(0), // R-Axis
() -> trueFunct(),
() -> swerveSubsystem.getHeading())); 
*/
    // DEBUG SETUP -> ZERO MOVEMENT
/*     swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> zeroFunct(), // X-Axis
    () -> rightJoystick.getRawAxis(2), // Y-Axis
    () -> zeroFunct(), // R-Axis
    () -> !leftJoystick.getRawButton(Constants.IOConstants.kFieldOrientedButton))); */
    
  //>--------------T-R-A-N-S-----------------//
    
  /* 
    swerveSubsystem.setDefaultCommand(new SwerveJoystick(swerveSubsystem,
    () -> tx16s.getRawAxis(0), // X-Axis
    () -> -tx16s.getRawAxis(1), // Y-Axis
    () -> tx16s.getRawAxis(3), // R-Axis
    () -> tx16s.getRawButton(0), // Field oriented -does nothing right now
    () -> swerveSubsystem.getHeading(), // Navx heading
    () -> tx16s.getRawButton(4))); // Flick offset button, should be toggle!
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

  //>----------S-E-N-D-E-R----------<//

  // Add auto commands to selector
    autoChooser.setDefaultOption("Forward", autoForward);
    autoChooser.addOption("AutoOne", autoOne);

  // Add auto chooser to smart dashboard
    SmartDashboard.putData(autoChooser);
  
  //>------------------------------<//

    // Run button binding method
    configureButtonBindings();
  }

  //------------------------------------D-E-B-U-G------------------------------------//

  private double zeroFunct(){return 0;}

  private boolean trueFunct(){return true;}

  //------------------------------------B-U-T-T-O-N-S------------------------------------//
 
  // Create buttons bindings
  private void configureButtonBindings(){

    // Open/Close grabber
    new JoystickButton(tx16s, 2).onTrue(new GrabberToggle(grabberSubsystem));
    new JoystickButton(tx16s, 2).onFalse(new GrabberToggle(grabberSubsystem));

    // Auto april tag align
    new JoystickButton(rightJoystick, 1).onTrue(new SwerveAlignBasic(swerveSubsystem, visionSubsystem,
      () -> swerveSubsystem.getHeading(), () -> rightJoystick.getRawButton(1), () -> tx16s.getRawAxis(5)));
// button 8 on tx16s
    // Run autonmous command during teleop
    //new JoystickButton(tx16s, 3).onTrue(new TrajectoryWeaver(swerveSubsystem,xController,yController,ppThetaController, pathOne, true));

  }

  //------------------------------------R-E-F-E-R-R-E-R-S------------------------------------//

    public void containerResetAllEncoders(){
      swerveSubsystem.resetAllEncoders();
    }

  //------------------------------------A-U-T-O-N-O-M-O-U-S------------------------------------//
  
  // Return the command to run during auto
  public Command getAutonomousCommand(){

    // Command to run
    Command autoCommand = null;

    return autoChooser.getSelected();
  }

}