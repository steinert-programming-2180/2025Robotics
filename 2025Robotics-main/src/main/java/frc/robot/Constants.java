package frc.robot;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.system.plant.DCMotor;



public final class Constants {

    public static final double stickDeadband = 0.1;
    public static final double ps5RumbleWarningTime = 10; // IN SECONDS

    
    public static final double ROBOT_MASS_KG = 23.35;
    public static final double ROBOT_RADIUS_M = 0.4939;
    public static final double ROBOT_MOI = 0.5 * ROBOT_MASS_KG * Math.pow(ROBOT_RADIUS_M, 2);
    public static final ModuleConfig MODULE_CONFIG = 
        new ModuleConfig(0.051, 2, 1.2, DCMotor.getNeoVortex(1).withReduction(6.75), 60, ROBOT_MASS_KG, 1);
    
    public static final RobotConfig PP_CONFIG = 
        new RobotConfig(ROBOT_MASS_KG, ROBOT_MOI, MODULE_CONFIG, getModuleTranslations());

    public static Translation2d[] getModuleTranslations() {
        return new Translation2d[] {
            new Translation2d(0.273, 0.273),
            new Translation2d(0.273, -0.273),
            new Translation2d(-0.273, 0.273),
            new Translation2d(-0.273, -0.273)
        };
    }



    public static final class PoseEstimator{
        public static final Matrix<N3, N1> stateStdDevs = VecBuilder.fill(0.1, 0.1, 0.1);
        public static final Matrix<N3, N1> VisionStdDevs = VecBuilder.fill(0.9, 0.9, 0.9);
    }

    public static final class OperatorConstants{
        public static final int PS5ControllerPort=1;
        public static final double PS5ControllerRumble = 1; // Intensity of controller rumbling
    }


    public static final class LimelightConstants {
        public static final double LIME_ANGLE = 0;
        public static final double LIME_HEIGHT = 0;
        public static final double LIME_TILTUP = 100;
        public static final double LIME_TILTDOWN = 55;
        public static final double LIME_TILTNEUTRAL = 80;
        public static final int LIME_SERVO = 1;
      }
    public static class FieldConstants{
        public static final double TARGET_MAX_HEIGHT = 0;
    }

    public static double inchesToMeters(double d){
        return d * 0.024;
    }

    public static class wristConstants{
        
        public static final int beamBreakSensorRioID = 0;

        public static final double wristRotationMotorkP = 0;
        public static final double wristRotationMotorkI = 0;
        public static final double wristRotationMotorkD = 0;


        public static final double wristMotorCoralSpeed = 0.5; // speed for having the wrist motor for intake
        public static final double wristMotorAlgaeSpeed = 0.5; // speed for having the algae motor for intake

        public static final int wristMotorRotation = 20;
        public static final int wristMotorCoral = 21;
        public static final int wristMotorAlgae = 22; 
        
        
    }

    public static class ArmConstants{

        public static final int armFollowMotorPort = 10;
        public static final int armBaseMotorPort = 11;
        public static final int armExtendMotorPort = 12;
        public static final int armWristMotorPort = 13;
    
        public static final int intakeMotorPort = 14;
        public static final int algaeMotorPort = 15;
    
        public static final double intakeSpeed = 0.8;
        public static final double algaeSpeed = 0.75;
    
    
        public static final double armBaseP=0.06;
        public static final double armBaseI=0;
        public static final double armBaseD=0.7;
    
        public static final double armExtendP=0.05;
        public static final double armExtendI=0;
        public static final double armExtendD=0;
    
        public static final double armWristP=0.05;
        public static final double armWristI=0;
        public static final double armWristD=0;

        public static final double armZeroPos=74.5;

        public static final int bottomLimitSwitchID=0;
        public static final int topLimitSwitchID=999;
    }
    public static class AutoConstans{

        public static final String parseErrorMessage = "JSON_CANNOT_BE_PARSED";

        public static final String ioExceptionError = "ERROR_GETTING_AUTO/PATH";
        
        public static final double algaeWaitTime = 0.0; //wait time in auto to do algae, probably after l4 score and aiming at algae
        public static final double kMaxSpeedMetersPerSecond = 9; // Set from 2024Robotics code, probably have to change
        public static final double kMaxAccelerationMetersPerSecondSquared = 6; // Set from 2024Robotics code, probably have to change
        public static final double kMaxAngularSpeedRadiansPerSecond = Math.PI; // Set from 2024Robotics code, probably have to change
        public static final double kMaxAngularSpeedRadiansPerSecondSquared = Math.PI; // Set from 2024Robotics code, probably have to change

        

    }
}