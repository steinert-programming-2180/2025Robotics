package frc.robot.subsystems;

import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.SparkMax;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.motorcontrol.Spark;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Endgame extends SubsystemBase{

DigitalInput EndgameimitSwitch = new DigitalInput(Constants.endgameConstants.endgameLimitSwitchID);

private SparkMax endgameSpinMotor;
private SparkMax endgameFollowMotor;
private SparkMax endgameRotationMotor; 

private SparkMaxConfig endgameSpinMotorConfig;
private SparkMaxConfig endgameFollowMotorConfig;
private SparkMaxConfig endGameRotationConfig;

    public Endgame(){


    endgameSpinMotor = new SparkMax(Constants.endgameConstants.endgameSpinMotor1, MotorType.kBrushless);
    endgameFollowMotor = new SparkMax(Constants.endgameConstants.endgameFollowMotor, MotorType.kBrushless);
    endgameRotationMotor = new SparkMax(Constants.endgameConstants.engameRotationMotor, MotorType.kBrushless);


    endgameSpinMotorConfig = new SparkMaxConfig();
    endgameFollowMotorConfig = new SparkMaxConfig();
    endGameRotationConfig= new SparkMaxConfig();

    configMotors();
    }

    public void configMotors(){

        endgameFollowMotorConfig.follow(endgameSpinMotor, false);

            // Endgame rotation motor configuration

        endGameRotationConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        endGameRotationConfig.absoluteEncoder
        .positionConversionFactor(360);
        // .velocityConversionFactor(360);

        endgameRotationMotor.configure(endGameRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Endgame spin motor configuration

        endgameSpinMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        endgameSpinMotorConfig.absoluteEncoder
        .positionConversionFactor(360);
        // .velocityConversionFactor(360);

        endgameSpinMotor.configure(endgameSpinMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

            // Endgame follow motor configuration

        endgameFollowMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        endgameFollowMotorConfig.absoluteEncoder
        .positionConversionFactor(360);
        // .velocityConversionFactor(360);

        endgameFollowMotor.configure(endgameFollowMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }


    public void EndgamePhaseTwo(){
        endgameRotationMotor.set(0.2);
    }

    public void endgamePhaseOne(){
        if(!EndgameimitSwitch.get()){
        endgameSpinMotor.set(0.2);
        }
    }
    
}
