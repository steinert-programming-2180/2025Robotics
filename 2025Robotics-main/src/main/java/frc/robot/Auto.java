package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.swerve.Swerve;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.swerve.SwerveModule;
// import edu.wpi.first.cameraserver.CameraServer;

import static edu.wpi.first.units.Units.Feet;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;




public class Auto extends SubsystemBase{

    private ChassisSpeeds desiredChassisSpeeds;
    private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4];
    private Swerve m_Swerve;
    private Limelight m_Limelight;
    private ChassisSpeeds updatedSpeeds;
    private Twist2d twistForPose;
    private Pose2d futureRobotPose;
    private RobotConfig m_config;
    private PoseEstimator m_Estimator;
    private Arm m_Arm;

    // PIDController xController = new PIDController(0, 0, 0);
    // PIDController yController = new PIDController(0, 0, 0);
    // ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0, null);


    // CameraServer.startAutomaticCapture();
    // NamedCommands.registerCommand("IntakeReverse", m_IntakeReverse); // pathplanner commands.

    // autoChooser = AutoBuilder.buildAutoChooser();
    // SmartDashboard.putData("Pick yo Auto big g", autoChooser);
    public SwerveModule[] mSwerveMods;
    // IntakeReverse m_IntakeReverse;
    // IntakeForward m_IntakeForward;
    // AlgaeForward m_AlgaeForward;
    // AlgaeReverse m_AlgaeReverse;


    
    private SendableChooser<Command> autoChooser;
    List<PathPlannerPath> pathGroup;

    public Auto(Swerve swerve, PoseEstimator m_Estimator, Arm m_Arm){
        this.m_Estimator = m_Estimator;
        this.m_Swerve = swerve;
        this.m_Arm = m_Arm;
        // m_config = new RobotConfig(23.35, 2.848, null, null)
        // NamedCommands.registerCommand("ScoreReef", m_IntakeReverse);
        // NamedCommands.registerCommand("Intaking", m_IntakeForward);
        // NamedCommands.registerCommand("Algae Forward", m_AlgaeForward);
        // NamedCommands.registerCommand("Algae Reverse", m_AlgaeReverse);
        NamedCommands.registerCommand("ScoreL4", new InstantCommand(() -> m_Arm.setAngle(75)));
        
        settingUpPathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

        // autoChooser.addOption("Left Path", new FollowPathCommand(null, null, null, null, null, m_config, null, m_Swerve));

        // try{
        //     pathGroup = PathPlannerAuto.getPathGroupFromAutoFile("BlueRightCoralScoreAuto");
        // } catch (IOException e){
        //     SmartDashboard.putString("IOException ERROR", "LOADING AUTO THAT DOESNT EXIST/CANNOT BE READ");    // im too lazy to try and make code to fix the issue
        // } catch(ParseException e){
        //     SmartDashboard.putString("PARSE EXCEPTION ERROR", "JSON FILE CANNOT BE PARSED");    // im too lazy to try and make code to fix the issue
        // }
    }



// Auto Commands used in auto on specific event points

    // public void autoCommands(){
    //     // m_IntakeForward = new PathPlannerAuto(m_IntakeForward);
    //     // m_IntakeForward.event("Intaking").onTrue(m_IntakeForward);

    //     // m_IntakeReverse = new PathPlannerAuto(m_AlgaeReverse);
    //     // m_IntakeReverse.event("ScoreReef").onTrue(m_IntakeReverse);


    //     // m_AlgaeReverse = new PathPlannerAuto(m_AlgaeReverse);
    //     // m_AlgaeReverse.event("ScoreReef").onTrue(m_AlgaeReverse);


    //     // // m_AlgaeForward = new PathPlannerAuto(m_AlgaeForward);
    //     // // m_AlgaeForward.timeElapsed(Constants.AutoConstans.algaeWaitTime).onTrue(m_AlgaeForward);
    // }


    public void settingUpPathPlanner(){
        try{

            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
            m_Estimator::getPose, // Robot pose supplier
            m_Estimator::resetOdometry,    // Method to reset odometry (will be called if your auto has a starting pose)
            m_Swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
            m_Swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
            new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                    new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                    new PIDConstants(5, 0.0, 0) // Rotation PID constants (old 0.03)
            ),
            config, 
            () -> {
                var alliance = DriverStation.getAlliance();
                if(alliance.isPresent()){
                    return alliance.get() == DriverStation.Alliance.Red;
                }
                return true;
            }, 
            m_Swerve
        );
        } catch (Exception ex){
            DriverStation.reportError("Failed to load pathplanner config and configure autobuilder", ex.getStackTrace());
        }

    
        // AutoBuilder.configure(
        //     m_Estimator::getPose, // Robot pose supplier
        //     m_Estimator::resetOdometry,    // Method to reset odometry (will be called if your auto has a starting pose)
        //     m_Swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
        //     m_Swerve::driveRobotRelative, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
        //     new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
        //             new PIDConstants(0.5, 0.0, 0.0), // Translation PID constants
        //             new PIDConstants(0.5, 0.0, 0.0) // Rotation PID constants (old 0.03)
        //     ),
        //     m_config, 
        //     () -> {
        //         Optional<Alliance> alliance = DriverStation.getAlliance();
        //         if(alliance.isPresent()){
        //             return alliance.get() == DriverStation.Alliance.Red;
        //         }
        //         return false;
        //     }, 
        //     m_Swerve
        // );
    }





// Already made in swerve.java

    // public void setModuleStates(SwerveModuleState[] desiredStates) {

    //    // System.out.println("setting module states: "+desiredStates[0]);
    //     SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);
        
    //     for(SwerveModule mod : mSwerveMods){
    //         mod.setDesiredState(desiredStates[mod.getModuleNumber()], true);
    //     }
    // }  




// Already made in swerve.java

    // public void autoDrive(ChassisSpeeds desiredChassisSpeeds) {
    //     // general swerve speeds --> speed per module
    //     swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds); 
    //     // toSwerveModuleStates(fieldRelativeSpeeds)
    //     setModuleStates(swerveModuleStates);
    //   }








    public Command getAutoCommand(){
        
        if(AutoBuilder.isConfigured()){
            return autoChooser.getSelected();
        }
        else{
            return Commands.none();
        }

    }
}
