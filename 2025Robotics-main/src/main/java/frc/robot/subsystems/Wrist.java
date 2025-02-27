package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;
import com.revrobotics.spark.config.SparkMaxConfig;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;


public class Wrist extends SubsystemBase{
public static double howMuchToPos;
private SparkMax wristMotorRotation;
private SparkMax wristMotorCoral;
private SparkMax wristMotorAlgae;
private SparkMaxConfig wristMotorRotationConfig;
private SparkMaxConfig wristMotorCoralConfig;
private SparkMaxConfig wristMotorAlgaeConfig;
public DigitalInput wristBeamBreak;



    public Wrist(){
        // wristBeamBreak = new DigitalInput(Constants.wristConstants.beamBreakSensorRioID);
        wristMotorRotation = new SparkMax (Constants.ArmConstants.armWristMotorPort, MotorType.kBrushless);
        wristMotorCoral = new SparkMax (Constants.wristConstants.wristMotorCoral, MotorType.kBrushless);
        wristMotorAlgae = new SparkMax (Constants.wristConstants.wristMotorAlgae, MotorType.kBrushless);

        wristMotorRotationConfig = new SparkMaxConfig();
        wristMotorAlgaeConfig = new SparkMaxConfig();
        wristMotorCoralConfig = new SparkMaxConfig();

        configureMotors();
    }
    
public void configureMotors(){
    // wrist rotation motor configuration 

    wristMotorRotationConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        wristMotorRotationConfig.absoluteEncoder
        .positionConversionFactor(360);

        wristMotorRotationConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(Constants.wristConstants.wristRotationMotorkP, Constants.wristConstants.wristRotationMotorkI , Constants.wristConstants.wristRotationMotorkD);

        wristMotorRotation.configure(wristMotorRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    // Wrist motor algae configuration

    wristMotorAlgaeConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        wristMotorAlgaeConfig.absoluteEncoder
        .positionConversionFactor(360);

        wristMotorAlgaeConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(Constants.wristConstants.wristRotationMotorkP, Constants.wristConstants.wristRotationMotorkI , Constants.wristConstants.wristRotationMotorkD);

        wristMotorAlgae.configure(wristMotorAlgaeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //Wrist motor coral configuration


    wristMotorCoralConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        wristMotorCoralConfig.absoluteEncoder
        .positionConversionFactor(360);

        wristMotorCoralConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(Constants.wristConstants.wristRotationMotorkP, Constants.wristConstants.wristRotationMotorkI , Constants.wristConstants.wristRotationMotorkD);

        wristMotorCoral.configure(wristMotorCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

@Override
public void periodic() {
    
}

public boolean getBeamBreak(){
    return wristBeamBreak.get();
}

    
public void rotateTheWrist(double wristAngle){

    howMuchToPos = Math.abs(wristMotorRotation.getAbsoluteEncoder().getPosition()-wristAngle);
    
    if(howMuchToPos>=0.1){
        wristMotorRotation.getClosedLoopController().setReference(wristAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        SmartDashboard.putNumber("Wrist PID Output", wristMotorRotation.getAppliedOutput());
    }
}

public void wristIntakeForwardCoral(){
    wristMotorCoral.set(Constants.wristConstants.wristMotorCoralSpeed);
}

public void wristMotorForwardAlgae(){
    wristMotorAlgae.set(Constants.wristConstants.wristMotorAlgaeSpeed);
}

public void wristIntakeBackwardCoral(){
    wristMotorCoral.set(-Constants.wristConstants.wristMotorCoralSpeed);
}

public void wristMotorBackwardAlgae(){
    wristMotorAlgae.set(-Constants.wristConstants.wristMotorAlgaeSpeed);
}


public void wristScoringStop(){
    // made so the scoring element motors will stop spinning. 
    wristMotorCoral.set(0);
    wristMotorAlgae.set(0);

}


}
