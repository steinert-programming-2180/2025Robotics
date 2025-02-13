package frc.robot;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;

 
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.networktables.NetworkTableEntry;


import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.hal.SimDevice.Direction;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.commands.TeleopSwerve;
import frc.robot.subsystems.Arm;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeReverse;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // private final RobotContainer m_RobotContainer = new RobotContainer();
    // private final Arm m_Arm = new Arm(); 
    // private final IntakeReverse m_IntakeReverse = new IntakeReverse(m_Arm);
    // private final IntakeForward m_IntakeForward = new IntakeForward(m_Arm);
    
    // private final Ps5Rumble m_Ps5Rumble = new Ps5Rumble(m_RobotContainer);
    // private final AutonomousCommand m_autonomousCommand = new AutonomousCommand(m_Swerve, m_Arm, m_IntakeForward, m_IntakeReverse);
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    // private final CommandPS5Controller PS5Controller=new CommandPS5Controller(OperatorConstants.PS5ControllerPort);

   /* Driver Controls */
	private final int translationAxis = PS5Controller.Axis.kLeftY.value;
	private final int strafeAxis = PS5Controller.Axis.kLeftX.value;
	private final int rotationAxis = PS5Controller.Axis.kRightX.value;

    /* Driver Buttons */
    private final JoystickButton zeroGyro = new JoystickButton(driver, PS5Controller.Button.kOptions.value);
    private final JoystickButton robotCentric = new JoystickButton(driver, PS5Controller.Button.kL1.value);

    private final JoystickButton dampen = new JoystickButton(driver, PS5Controller.Button.kR1.value);

    private final POVButton up = new POVButton(driver, 90);
    private final POVButton down = new POVButton(driver, 270);
    private final POVButton right = new POVButton(driver, 180);
    private final POVButton left = new POVButton(driver, 0);

    private final JoystickButton square = new JoystickButton(driver, PS5Controller.Button.kSquare.value);
    private final JoystickButton triangle = new JoystickButton(driver, PS5Controller.Button.kTriangle.value);
    private final JoystickButton circle = new JoystickButton(driver, PS5Controller.Button.kCircle.value);
    private final JoystickButton cross = new JoystickButton(driver, PS5Controller.Button.kCross.value);
    
    CommandPS5Controller m_ps5driverController = new CommandPS5Controller(Constants.OperatorConstants.PS5ControllerPort);


    /* Subsystems */
    private final Swerve s_Swerve = new Swerve();
    private final PoseEstimator s_PoseEstimator = new PoseEstimator();

    private final Auto m_auto = new Auto(s_Swerve);

    Trigger rumblePS5Trigger;

    // The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    
        DataLogManager.getLog();


        // rumblePS5Trigger = new Trigger(DriverStation::isTeleopEnabled).onTrue(Commands.waitSeconds(Constants.ps5RumbleWarningTime).andThen(m_Ps5Rumble));
        

        s_Swerve.setDefaultCommand(
            new TeleopSwerve(
                s_Swerve, 
                () -> -driver.getRawAxis(translationAxis), 
                () -> -driver.getRawAxis(strafeAxis), 
                () -> -driver.getRawAxis(rotationAxis), 
                () -> false,
                () -> dampen.getAsBoolean(),
                () -> 1 //speed multiplier 
            )
        );

        SmartDashboard.putNumber("Rotation Controller Axis", rotationAxis);


        // Configure the button bindings
        configureButtonBindings();
    }

    /**
     * Use this method to define your button->command mappings. Buttons can be created by
     * instantiating a {@link GenericHID} or one of its subclasses ({@link
     * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
     * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
     */
    private void configureButtonBindings() {
        /* Driver Buttons */
        zeroGyro.onTrue(new InstantCommand(() -> s_Swerve.zeroGyro()));


        //heading lock bindings
        up.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d90)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        left.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d180)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        right.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d0)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );
        down.onTrue(
            new InstantCommand(() -> States.driveState = States.DriveStates.d270)).onFalse(
            new InstantCommand(() -> States.driveState = States.DriveStates.standard)
            );


        //square.onTrue(s_Swerve.sysIdDynamic(s_Swerve.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)));
        // why?
        square.onTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        circle.onTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        triangle.onTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        cross.onTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));
    }

    public void ps5ControllerRumble(){
        m_ps5driverController.setRumble(GenericHID.RumbleType.kBothRumble, Constants.OperatorConstants.PS5ControllerRumble);
    }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_auto.getAutoCommand();
        
    }
}