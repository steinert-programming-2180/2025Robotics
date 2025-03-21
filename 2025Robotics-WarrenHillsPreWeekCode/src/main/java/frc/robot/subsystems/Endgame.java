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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;

public class Endgame extends SubsystemBase{

DigitalInput EndgameimitSwitch = new DigitalInput(Constants.EndgameConstants.endgameLimitSwitchID);

private SparkMax endgameSpinMotor;
private SparkMax endgameRotationMotor; 

private SparkMaxConfig endgameSpinMotorConfig;
private SparkMaxConfig endGameRotationConfig;

    public Endgame(){


    endgameSpinMotor = new SparkMax(Constants.EndgameConstants.endgameSpinMotor1, MotorType.kBrushless);
    endgameRotationMotor = new SparkMax(Constants.EndgameConstants.engameRotationMotor, MotorType.kBrushless);


    endgameSpinMotorConfig = new SparkMaxConfig();
    endGameRotationConfig= new SparkMaxConfig();

    configMotors();
    }

    public void configMotors(){

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

    }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Endgame deploy motor", endgameSpinMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putNumber("Endgame rotation motor", endgameRotationMotor.getAbsoluteEncoder().getPosition());
    }


    public void endgamePhaseTwo(){
        endgameRotationMotor.set(0.75);
    }

    public void endgamePhaseOne(){
        endgameSpinMotor.set(0.125);
    }

    public void reverseClimber(){
        endgameSpinMotor.set(-0.125);
    }

    public void reverseRotation(){
        endgameRotationMotor.set(-0.75);
    }

    public void stopRotation(){
        endgameRotationMotor.set(0.0);
    }
    
}
