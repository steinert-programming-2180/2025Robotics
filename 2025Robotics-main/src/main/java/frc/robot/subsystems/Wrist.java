package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
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
import edu.wpi.first.wpilibj.DutyCycleEncoder;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.math.Conversions;
import frc.robot.Constants;
import frc.robot.Constants.ArmConstants;


public class Wrist extends SubsystemBase{
public static double howMuchToPos;
// private SparkMax wristMotorRotation;
private SparkMax wristMotorCoral;
private SparkMaxConfig wristMotorRotationConfig;
private SparkMaxConfig wristMotorCoralConfig;
public DigitalInput wristBeamBreak;
private Encoder wristEncoder;
private double kP, kI, kD;
private double wristPos0 = 105;
// private DigitalInput EncoderLeft;
// private DigitalInput EncoderRight;



    public Wrist(){
        // wristBeamBreak = new DigitalInput(Constants.wristConstants.beamBreakSensorRioID);
        // wristMotorRotation = new SparkMax (Constants.ArmConstants.armWristMotorPort, MotorType.kBrushless);
        wristMotorCoral = new SparkMax (Constants.ArmConstants.intakeMotorPort, MotorType.kBrushless);

        wristMotorRotationConfig = new SparkMaxConfig();
        wristMotorCoralConfig = new SparkMaxConfig();

        // EncoderLeft = new DigitalInput(2);
        // EncoderRight = new DigitalInput(3);
        wristEncoder = new Encoder(6, 7);
        configureMotors();
    }
    
public void configureMotors(){
    // wrist rotation motor configuration 

    SmartDashboard.putNumber("Wrist kP", Constants.wristConstants.wristRotationMotorkP);
    SmartDashboard.putNumber("Wrist kI", Constants.wristConstants.wristRotationMotorkI);
    SmartDashboard.putNumber("Wrist kD", Constants.wristConstants.wristRotationMotorkD);

    wristMotorRotationConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);

        wristMotorRotationConfig.alternateEncoder
        .positionConversionFactor(360)
        .setSparkMaxDataPortConfig();

        wristMotorRotationConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAlternateOrExternalEncoder)
        .pid(Constants.wristConstants.wristRotationMotorkP, Constants.wristConstants.wristRotationMotorkI , Constants.wristConstants.wristRotationMotorkD);

    // wristMotorRotation.configure(wristMotorRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    //Wrist motor coral configuration


    wristMotorCoralConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    wristMotorCoral.configure(wristMotorCoralConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


    }

public boolean getBeamBreak(){
    return wristBeamBreak.get();
}

@Override
    public void periodic() {
        SmartDashboard.putNumber("wrist pos", wristEncoder.get());

        this.kP = SmartDashboard.getNumber("Wrist kP", Constants.wristConstants.wristRotationMotorkP);
        this.kI = SmartDashboard.getNumber("Wrist kI", Constants.wristConstants.wristRotationMotorkI);
        this.kD = SmartDashboard.getNumber("Wrist kD", Constants.wristConstants.wristRotationMotorkD);
    }
    
// public void rotateTheWrist(double wristAngle){

//     howMuchToPos = Math.abs(wristMotorRotation.getAbsoluteEncoder().getPosition()-wristAngle);

//     wristMotorRotationConfig.closedLoop.pid(this.kP, this.kI, this.kD);
//     wristMotorRotation.configure(wristMotorRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    
//     if(howMuchToPos>=0.1){
//         wristMotorRotation.getClosedLoopController().setReference(wristAngle - wristPos0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
//         SmartDashboard.putNumber("Wrist PID Output", wristMotorRotation.getAppliedOutput());
//     }
// }

public void rotateTheWrist(double wristAngle){

    // wristMotorRotationConfig.closedLoop.pid(this.kP, this.kI, this.kD);
    // wristMotorRotation.configure(wristMotorRotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
        
    // wristMotorRotation.getClosedLoopController().setReference(wristAngle - wristPos0, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    // SmartDashboard.putNumber("Wrist PID Output", wristMotorRotation.getAppliedOutput());
}

public void activateIntake(){
    wristMotorCoral.set(Constants.wristConstants.wristMotorCoralSpeed);
}

// public void wristMotorForwardAlgae(){
//     wristMotorAlgae.set(Constants.wristConstants.wristMotorAlgaeSpeed);
// }

public void reverseIntake(){
    wristMotorCoral.set(-Constants.wristConstants.wristMotorCoralSpeed);
}

// public void wristMotorBackwardAlgae(){
//     wristMotorAlgae.set(-Constants.wristConstants.wristMotorAlgaeSpeed);
// }


public void wristScoringStop(){
    // made so the scoring element motors will stop spinning. 
    wristMotorCoral.set(0);
    // wristMotorAlgae.set(0);

}


}
