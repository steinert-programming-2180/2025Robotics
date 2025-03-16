package frc.robot.subsystems.swerve;

import static edu.wpi.first.units.Units.Volts;

import java.util.List;
import java.util.concurrent.locks.Lock;
import java.util.concurrent.locks.ReentrantLock;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.inputs.LoggableInputs;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
// import com.ctre.phoenix.sensors.Pigeon2;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.StructArrayPublisher;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj.Alert;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;
import frc.lib.math.GeometryUtils;
import frc.robot.Constants;
import frc.robot.Constants.Mode;
import frc.robot.SwerveConstants;
import frc.robot.subsystems.Limelight;
import frc.robot.subsystems.PoseEstimator;
import frc.robot.subsystems.swerve.GyroIO.GyroIOInputs;


public class Swerve extends SubsystemBase {

    public SwerveDriveOdometry swerveOdometry;
    public SwerveMod[] mSwerveMods;
    private final AHRS gyro = new AHRS(NavXComType.kMXP_SPI);

    //public Pigeon2 gyro;

    private ChassisSpeeds desiredChassisSpeeds;
    private SwerveModuleState[] swerveModuleStates = new SwerveModuleState[4]; 
    private SwerveMod frontLeft, frontRight, backLeft, backRight;
    private Limelight m_Limelight;
    private ChassisSpeeds updatedSpeeds;
    private Twist2d twistForPose;
    private Pose2d futureRobotPose;
    private RobotConfig m_Config;
    PIDController xController = new PIDController(0, 0, 0);
    PIDController yController = new PIDController(0, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(0, 0, 0, null);
    
    static final Lock odometryLock = new ReentrantLock();
    private final GyroIO gyroIO;
    private final GyroIOInputsAutoLogged gyroInputs = new GyroIOInputsAutoLogged();
    private final Alert gyroDisconnectedAlert = new Alert("Disconnected gyro, using kinematics as fallback.", AlertType.kError);
    private Rotation2d rawGyroRotation = new Rotation2d();
    private SwerveModulePosition[] lastModulePositions = // For delta tracking
        new SwerveModulePosition[] {
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition(),
            new SwerveModulePosition()
        };
    private SwerveDrivePoseEstimator poseEstimator =
      new SwerveDrivePoseEstimator(SwerveConfig.swerveKinematics, rawGyroRotation, lastModulePositions, new Pose2d()); 

    private final Module[] modules = new Module[4]; // FL, FR, BL, BR

    final StructArrayPublisher<SwerveModuleState> desiredStateLogger =
        NetworkTableInstance.getDefault().getStructArrayTopic("DesiredState",
        SwerveModuleState.struct).publish();
    final StructArrayPublisher<SwerveModuleState> actualStateLogger =
        NetworkTableInstance.getDefault().getStructArrayTopic("ActualState",
        SwerveModuleState.struct).publish();

    
    public Swerve(GyroIO gyroIO, Limelight m_Limelight, ModuleIO flModuleIO, ModuleIO frModuleIO, ModuleIO blModuleIO, ModuleIO brModuleIO) {

        this.gyroIO = gyroIO;
        modules[0] = new Module(flModuleIO, 0);
        modules[1] = new Module(frModuleIO, 1);
        modules[2] = new Module(blModuleIO, 2);
        modules[3] = new Module(brModuleIO, 3);

        this.m_Limelight = m_Limelight;
        this.m_Config = Constants.PP_CONFIG;  

        frontLeft = new SwerveMod(0, SwerveConstants.Swerve.Mod0.constants);
        frontRight = new SwerveMod(1, SwerveConstants.Swerve.Mod1.constants);
        backLeft = new SwerveMod(2, SwerveConstants.Swerve.Mod2.constants);
        backRight = new SwerveMod(3, SwerveConstants.Swerve.Mod3.constants);

        mSwerveMods = new SwerveMod[] {
            frontLeft, frontRight, backLeft, backRight
        };

        swerveOdometry = new SwerveDriveOdometry(SwerveConfig.swerveKinematics, getYaw(), getModulePositions());
        zeroGyro();


        //     var alliance = DriverStation.getAlliance();
        //     if(alliance.isPresent()){
        //         return alliance.get() == DriverStation.Alliance.Red;
        //     }
        //     return false;
        // }, this

        
    }

    public final SysIdRoutine sysIdRoutine = new SysIdRoutine(
        new SysIdRoutine.Config(
            // null,
            // null,
            // null,
            // (state) -> Logger.recordOutput("Drive/SysIdState", state.toString())
        ),
        new SysIdRoutine.Mechanism(
            (Voltage voltage) -> {
                for (SwerveModule mod : mSwerveMods) {
                    mod.getDriveMotor().setVoltage(voltage.in(Volts) / 4);
                    Logger.recordOutput("Drive/SysIdVelocity", mod.getDriveMotor().getAppliedOutput());
                    // mod.getAngleMotor().setVoltage(voltage.in(Volts));
                }
            },
            null, 
            this
        )
    );


    // public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.quasistatic(direction);
    // }

    // public Command sysIdDynamic(SysIdRoutine.Direction direction) {
    //     return sysIdRoutine.dynamic(direction);
    // }

    // public Command sysIdQuasistaticForward(SysIdRoutine.Direction kForward){
    //     return sysIdRoutine.quasistatic(SysIdRoutine.Direction.kForward);
    // }

    public Command sysIdQuasistatic(SysIdRoutine.Direction direction){
        return sysIdRoutine.quasistatic(direction);
    }

    public Command sysIdDynamic(SysIdRoutine.Direction direction) {
        return sysIdRoutine.dynamic(direction);
    }

    private ChassisSpeeds correctForDynamics(ChassisSpeeds originalSpeeds) {
        final double LOOP_TIME_S = 0.02;
        futureRobotPose =
            new Pose2d(
                originalSpeeds.vxMetersPerSecond * LOOP_TIME_S,
                originalSpeeds.vyMetersPerSecond * LOOP_TIME_S,
                Rotation2d.fromRadians(originalSpeeds.omegaRadiansPerSecond * LOOP_TIME_S));
        twistForPose = GeometryUtils.log(futureRobotPose);
        updatedSpeeds =
            new ChassisSpeeds(
                twistForPose.dx / LOOP_TIME_S,
                twistForPose.dy / LOOP_TIME_S,
                twistForPose.dtheta / LOOP_TIME_S);
        return updatedSpeeds;
    }
    
    @Override
    public void periodic() {
        odometryLock.lock(); // Prevents odometry updates while reading data
        gyroIO.updateInputs(gyroInputs);
        Logger.processInputs("Drive/Gyro", gyroInputs);
        for (var module : modules) {
        module.periodic();
        }
        odometryLock.unlock();

        // Stop moving when disabled
        if (DriverStation.isDisabled()) {
        for (var module : modules) {
            module.stop();
        }
        }

        // Log empty setpoint states when disabled
        if (DriverStation.isDisabled()) {
        Logger.recordOutput("SwerveStates/Setpoints", new SwerveModuleState[] {});
        Logger.recordOutput("SwerveStates/SetpointsOptimized", new SwerveModuleState[] {});
        }

        // Update odometry
        double[] sampleTimestamps =
            modules[0].getOdometryTimestamps(); // All signals are sampled together
        int sampleCount = sampleTimestamps.length;
        for (int i = 0; i < sampleCount; i++) {
        // Read wheel positions and deltas from each module
        SwerveModulePosition[] modulePositions = new SwerveModulePosition[4];
        SwerveModulePosition[] moduleDeltas = new SwerveModulePosition[4];
        for (int moduleIndex = 0; moduleIndex < 4; moduleIndex++) {
            modulePositions[moduleIndex] = modules[moduleIndex].getOdometryPositions()[i];
            moduleDeltas[moduleIndex] =
                new SwerveModulePosition(
                    modulePositions[moduleIndex].distanceMeters
                        - lastModulePositions[moduleIndex].distanceMeters,
                    modulePositions[moduleIndex].angle);
            lastModulePositions[moduleIndex] = modulePositions[moduleIndex];
        }

        // Update gyro angle
        if (gyroInputs.connected) {
            // Use the real gyro angle
            rawGyroRotation = gyroInputs.odometryYawPositions[i];
        } else {
            // Use the angle delta from the kinematics and module deltas
            Twist2d twist = SwerveConfig.swerveKinematics.toTwist2d(moduleDeltas);
            rawGyroRotation = rawGyroRotation.plus(new Rotation2d(twist.dtheta));
        }

        // Apply update
        poseEstimator.updateWithTime(sampleTimestamps[i], rawGyroRotation, modulePositions);
        }

        // Update gyro alert
        gyroDisconnectedAlert.set(!gyroInputs.connected && Constants.currentMode != Mode.SIM);

    }

    public void driveRobotRelative(ChassisSpeeds desiredChassisSpeeds) {
        
        SmartDashboard.putNumber("desired vx (m/s)", desiredChassisSpeeds.vxMetersPerSecond);
        SmartDashboard.putNumber("desired vy (m/s)", desiredChassisSpeeds.vyMetersPerSecond);

        // general swerve speeds --> speed per module
        // ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);
        swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds); 
        // SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConfig.maxSpeed);
        // toSwerveModuleStates(fieldRelativeSpeeds)
        setModuleStates(swerveModuleStates);
      }

    public void runVelocity(ChassisSpeeds desiredChassisSpeeds) {
        // Calculate module setpoints
        ChassisSpeeds discreteSpeeds = ChassisSpeeds.discretize(desiredChassisSpeeds, 0.02);
        SwerveModuleState[] setpointStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(discreteSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(setpointStates, SwerveConfig.maxSpeed);

        // Log unoptimized setpoints
        Logger.recordOutput("SwerveStates/Setpoints", setpointStates);
        Logger.recordOutput("SwerveChassisSpeeds/Setpoints", discreteSpeeds);

        // Send setpoints to modules
        for (int i = 0; i < 4; i++) {
            modules[i].runSetpoint(setpointStates[i]);
        }

        // Log optimized setpoints (runSetpoint mutates each state)
        Logger.recordOutput("SwerveStates/SetpointsOptimized", setpointStates);
    }



    public void teleopDrive(Translation2d translation, double rotation, boolean fieldRelative, boolean isOpenLoop) {
        desiredChassisSpeeds =
        fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
        translation.getX(),
        translation.getY(),
        rotation,
        getYaw())
        : new ChassisSpeeds(
                translation.getX(),
                translation.getY(),
                rotation);
        desiredChassisSpeeds = correctForDynamics(desiredChassisSpeeds);

        swerveModuleStates = SwerveConfig.swerveKinematics.toSwerveModuleStates(desiredChassisSpeeds);
        SwerveDriveKinematics.desaturateWheelSpeeds(swerveModuleStates, SwerveConfig.maxSpeed);
        
        for(SwerveModule mod : mSwerveMods){
            mod.setDesiredState(swerveModuleStates[mod.getModuleNumber()], isOpenLoop);
        }

    }    
    /* Used by SwerveControllerCommand in Auto */
    public void setModuleStates(SwerveModuleState[] desiredStates) {

       // System.out.println("setting module states: "+desiredStates[0]);
        SwerveDriveKinematics.desaturateWheelSpeeds(desiredStates, SwerveConfig.maxSpeed);
        
        for(SwerveMod mod : mSwerveMods){
            mod.setDesiredState(desiredStates[mod.getModuleNumber()]);
        }

        desiredStateLogger.set(desiredStates);
        actualStateLogger.set(getModuleStates());
    }  
    // public ChassisSpeeds getRobotRelativSpeeds(){
    //     return SwerveConfig.swerveKinematics.toChassisSpeeds(swerveModuleStates);
    // }
    
    // public Pose2d getPose() {
    //     Pose2d p =  swerveOdometry.getPoseMeters();
    //     return new Pose2d(-p.getX(),-p.getY(),  p.getRotation());
    // }
    
    // public void resetOdometry(Pose2d pose) {
        
    //     swerveOdometry.resetPosition(new Rotation2d(), getModulePositions(), pose);
    //     zeroGyro(pose.getRotation().getDegrees());
       
    // }
    public SwerveModuleState[] getModuleStates() {
        SwerveModuleState[] states = new SwerveModuleState[4];
        for(SwerveMod mod : mSwerveMods) {
            states[mod.getModuleNumber()] = mod.getState();
        }
        return states;
    }

    public SwerveModulePosition[] getModulePositions() {
        SwerveModulePosition[] positions = new SwerveModulePosition[4];
        for(SwerveMod mod : mSwerveMods) {
            positions[mod.getModuleNumber()] = mod.getPosition();
        }
        return positions;
    }
    
    public ChassisSpeeds getRobotRelativeSpeeds(){
        return SwerveConfig.swerveKinematics.toChassisSpeeds(
            frontLeft.getState(),
            frontRight.getState(),
            backLeft.getState(),
            backRight.getState()
        );
        // return ChassisSpeeds.fromFieldRelativeSpeeds(SwerveConfig.swerveKinematics.toChassisSpeeds(getModuleStates()));
    }

    public void zeroGyro(double deg) {
        if(SwerveConfig.invertGyro) {
            deg = -deg;
        }
        gyro.reset();
        swerveOdometry.update(getYaw(), getModulePositions());  
    }

    public void zeroGyro() {  
       zeroGyro(0);
    }

    public Rotation2d getYaw() {
        return (SwerveConfig.invertGyro) ? Rotation2d.fromDegrees(360 - gyro.getAngle()) : Rotation2d.fromDegrees(gyro.getAngle());
    }

    
    
};