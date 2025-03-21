package frc.robot;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.commands.PathPlannerAuto;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.pathplanner.lib.util.PathPlannerLogging;

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
import frc.robot.util.LocalADStarAK;

import static edu.wpi.first.units.Units.Feet;

import java.util.List;
import java.util.Optional;

import org.littletonrobotics.junction.Logger;
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
    public SwerveModule[] mSwerveMods;
    private SendableChooser<Command> autoChooser;


    public Auto(Swerve swerve, PoseEstimator m_Estimator, Arm m_Arm){
        this.m_Estimator = m_Estimator;
        this.m_Swerve = swerve;
        this.m_Arm = m_Arm;

        NamedCommands.registerCommand("ScoreL4", new InstantCommand(() -> m_Arm.setAngle(75)));
        
        settingUpPathPlanner();
        autoChooser = AutoBuilder.buildAutoChooser();
        SmartDashboard.putData("Auto Chooser", autoChooser);

    }


    public void settingUpPathPlanner(){
        try{

            var config = RobotConfig.fromGUISettings();
            AutoBuilder.configure(
                m_Swerve::getPose, // Robot pose supplier
                m_Swerve::resetOdometry,    // Method to reset odometry (will be called if your auto has a starting pose)
                m_Swerve::getRobotRelativeSpeeds, // ChassisSpeeds supplier. MUST BE ROBOT RELATIVE
                m_Swerve::runVelocity, // Method that will drive the robot given ROBOT RELATIVE ChassisSpeeds. Also optionally outputs individual module feedforwards
                new PPHolonomicDriveController( // PPHolonomicController is the built in path following controller for holonomic drive trains
                        new PIDConstants(5, 0.0, 0.0), // Translation PID constants
                        new PIDConstants(0, 0.0, 0) // Rotation PID constants (old 0.03)
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

            Pathfinding.setPathfinder(new LocalADStarAK());
            PathPlannerLogging.setLogActivePathCallback(
                (activePath) -> {
                    Logger.recordOutput(
                        "Odometry/Trajectory", activePath.toArray(new Pose2d[activePath.size()]));
                });
            PathPlannerLogging.setLogTargetPoseCallback(
                (targetPose) -> {
                    Logger.recordOutput("Odometry/TrajectorySetpoint", targetPose);
                });
        } catch (Exception ex){
            DriverStation.reportError("Failed to load pathplanner config and configure autobuilder", ex.getStackTrace());
        }
    }

    public Command getAutoCommand(){
        
        if(AutoBuilder.isConfigured()){
            return autoChooser.getSelected();
        }
        else{
            return Commands.none();
        }

    }
}
