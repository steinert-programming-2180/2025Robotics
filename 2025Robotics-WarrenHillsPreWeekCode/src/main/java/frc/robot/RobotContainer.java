package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.PS5Controller;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.robot.Constants.ArmConstants;
import frc.robot.commands.TeleopSwerve;
import frc.robot.commands.ps5Rumble;
// import frc.robot.commands.TopTilt;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Endgame;
// import frc.robot.subsystems.Endgame;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.Manipulator;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.Wrist;
import frc.robot.subsystems.swerve.GyroIO;
import frc.robot.subsystems.swerve.ModuleIO;
import frc.robot.subsystems.swerve.ModuleIOSim;
import frc.robot.subsystems.swerve.Swerve;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
    // private final RobotContainer m_RobotContainer = new RobotContainer();
    
    private final Wrist wrist = new Wrist();
    private final Arm arm=new Arm();
    private final Limelight m_Limelight = new Limelight();
    private final Swerve s_Swerve = new Swerve(new GyroIO() {}, m_Limelight, new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim(), new ModuleIOSim());
    private final PoseEstimator m_PoseEstimator = new PoseEstimator(s_Swerve);
    private final Endgame climber=new Endgame();
    private final Manipulator manipulator=new Manipulator(arm, wrist);
    // private final IntakeReverse m_IntakeReverse = new IntakeReverse(m_Arm);
    // private final IntakeForward m_IntakeForward = new IntakeForward(m_Arm);
    
    private final ps5Rumble m_Ps5Rumble = new ps5Rumble();

    // private final AutonomousCommand m_autonomousCommand = new AutonomousCommand(m_Swerve, m_Arm, m_IntakeForward, m_IntakeReverse);
    
    /* Controllers */
    private final Joystick driver = new Joystick(0);
    private final Joystick operator = new Joystick(1);
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

    public final JoystickButton leftBumper = new JoystickButton(driver, PS5Controller.Button.kL1.value);
    public final JoystickButton rightBumper = new JoystickButton(driver, PS5Controller.Button.kR1.value);

    private final JoystickButton leftTrigger = new JoystickButton(driver, PS5Controller.Button.kL2.value);
    private final JoystickButton rightTrigger = new JoystickButton(driver, PS5Controller.Button.kR2.value);

    private final JoystickButton square = new JoystickButton(driver, PS5Controller.Button.kSquare.value);
    private final JoystickButton triangle = new JoystickButton(driver, PS5Controller.Button.kTriangle.value);
    private final JoystickButton circle = new JoystickButton(driver, PS5Controller.Button.kCircle.value);
    private final JoystickButton cross = new JoystickButton(driver, PS5Controller.Button.kCross.value);

    private final Auto m_Auto = new Auto(s_Swerve, m_PoseEstimator,arm);

    // private final SetBaseToAngle setBase40Degrees = new SetBaseToAngle(m_Arm, 40.0);
    // private final SetBaseToAngle setBase180Degrees = new SetBaseToAngle(m_Arm, 180);
    // private final SetBaseToAngle setBase270Degrees = new SetBaseToAngle(m_Arm, 270);
    // private final SetBaseToAngle setBase360Degrees = new SetBaseToAngle(m_Arm, 360);
    
    public static final CommandPS5Controller m_ps5driverController = new CommandPS5Controller(Constants.OperatorConstants.PS5ControllerPort);
    CommandXboxController xBoxController = new CommandXboxController(1);

    /* Subsystems */

    // private final Auto m_auto = new Auto(s_Swerve, m_Limelight);

    Trigger rumblePS5Trigger;

    // The container for the robot. Contains subsystems, OI devices, and commands. */
    public RobotContainer() {
    
        DataLogManager.getLog();

        // autoChooser = AutoBuilder.buildAutoChooser();
        // SmartDashboard.putData("Auto Chooser", autoChooser);


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
        // new Trigger(LimeController::getYButtonPressed).onTrue(new IncrementServo(m_Limelight));




        


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

        //PID testing
        square.onTrue(new InstantCommand(() -> wrist.setAngle(15)));
        triangle.onTrue(new InstantCommand(() -> wrist.setAngle(55)));


        //square.onTrue(s_Swerve.sysIdDynamic(s_Swerve.sysIdRoutine.dynamic(SysIdRoutine.Direction.kForward)));
        // why?
        // square.onTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kForward));
        // circle.onTrue(s_Swerve.sysIdDynamic(SysIdRoutine.Direction.kReverse));
        // triangle.onTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kForward));
        // cross.onTrue(s_Swerve.sysIdQuasistatic(SysIdRoutine.Direction.kReverse));

        // xBoxController.x().whileTrue(new InstantCommand(() -> m_Arm.raiseArm())).whileFalse(new InstantCommand(() -> m_Arm.stopRotating()));
        // xBoxController.x().toggleOnTrue(new InstantCommand(() -> m_Arm.lowerArm())).toggleOnFalse(new InstantCommand(() -> m_Arm.stopRotating()));
        
        // cross.onFalse(new InstantCommand(() -> m_Arm.stopRetract()));

        // if (leftBumper.getAsBoolean()) {
        //     m_Arm.rotate(0.2);
        // } else if (rightBumper.getAsBoolean()) {
        //     m_Arm.rotate(-0.2);
        // } else {
        //     m_Arm.stopRotating();
        // }
        // leftBumper.whileTrue(new InstantCommand(() -> m_Arm.rotate(0.2)));
        // rightBumper.whileTrue(new InstantCommand(() -> m_Arm.rotate(-0.2)));

        // circle.onTrue(new InstantCommand(() -> m_Arm.setAngle(75)));
        // square.whileTrue(new InstantCommand(() -> m_Arm.raiseArm()));
        // triangle.onTrue(new InstantCommand(() -> m_Arm.setAngle(90)));

        // circle extend
        // Cross retract
        
        
        // circle.onTrue(new InstantCommand(() -> m_Wrist.rotateTheWrist(20)));
        // triangle.onTrue(new InstantCommand(() -> m_Wrist.rotateTheWrist(45)));

        //reef lvl 1
        xBoxController.b().onTrue(new InstantCommand(() -> manipulator.reefLvl1()));
        // xBoxController.b().onTrue(new InstantCommand(() -> arm.setAngle(60)));
        //reef lvl 2 
        xBoxController.x().onTrue(new InstantCommand(() -> manipulator.reefLvl2()));
        //reef lvl 3 
        xBoxController.a().onTrue(new InstantCommand(() -> manipulator.reefLvl3()));
        // xBoxController.a().onTrue(new InstantCommand(() -> arm.setAngle(30)));
        //reef lvl 4 
        xBoxController.y().onTrue(new InstantCommand(() -> manipulator.reefLvl4()));
        //human player station
        xBoxController.start().onTrue(new InstantCommand(() -> manipulator.humanPlayer()));
        
        //Spin algae motor
        leftBumper.toggleOnTrue
        (new InstantCommand(() -> wrist.spinAlgae()))
        .toggleOnFalse(new InstantCommand(() -> wrist.stopAlgae()));
        //Reverse algae motor
        rightBumper.toggleOnTrue
        (new InstantCommand(() -> wrist.reverseAlgae()))
        .toggleOnFalse(new InstantCommand(() -> wrist.stopAlgae()));
        
        //Intake in
        leftTrigger.toggleOnTrue
        (new InstantCommand(() -> wrist.activateIntake()))
        .toggleOnFalse(new InstantCommand(() -> wrist.stopIntake()));
        //Intake out
        rightTrigger.toggleOnTrue
        (new InstantCommand(() -> wrist.reverseIntake()))
        .toggleOnFalse(new InstantCommand(() -> wrist.stopIntake()));

        //deploy climber (left on d pad)

        //retract climber (right on d pad)
        


        //manual overrides
        // wrist rotate up
        xBoxController.leftTrigger().toggleOnTrue
        (new InstantCommand(() -> wrist.manualRotate()))
        .toggleOnFalse(new InstantCommand(() -> wrist.stopRotation()));

        //wrist rotate down
        xBoxController.rightTrigger().toggleOnTrue
        (new InstantCommand(() -> wrist.manualReverseRotate()))
        .toggleOnFalse(new InstantCommand(() -> wrist.stopRotation()));

        //extend
        xBoxController.povUp().toggleOnTrue
        (new InstantCommand(() -> arm.extendArm()))
        .toggleOnFalse(new InstantCommand(() -> arm.stopRetract()));

        //retract
        xBoxController.povDown().toggleOnTrue
        (new InstantCommand(() -> arm.retractArm()))
        .toggleOnFalse(new InstantCommand(() -> arm.stopRetract()));

        //rotate arm up
        xBoxController.leftBumper().toggleOnTrue
        (new InstantCommand(() -> arm.raiseArm()))
        .toggleOnFalse(new InstantCommand(() -> arm.stopRotating()));

        //rotate arm down
        xBoxController.rightBumper().toggleOnTrue
        (new InstantCommand(() -> arm.lowerArm()))
        .toggleOnFalse(new InstantCommand(() -> arm.stopRotating()));
    }

    // public void ps5ControllerRumble(){
    //     m_ps5driverController.setRumble(GenericHID.RumbleType.kBothRumble, Constants.OperatorConstants.PS5ControllerRumble);
    // }

    /**
     * Use this to pass the autonomous command to the main {@link Robot} class.
     *
     * @return the command to run in autonomous
     */
    public Command getAutonomousCommand() {
        return m_Auto.getAutoCommand(); 
    }
}