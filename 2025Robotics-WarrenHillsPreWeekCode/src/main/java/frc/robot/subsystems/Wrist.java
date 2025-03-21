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
import frc.robot.Constants.WristConstants;


public class Wrist extends SubsystemBase{
public static double howMuchToPos;
private SparkMax algaeMotor;
private SparkMax intakeMotor;
private SparkMax rotationMotor;
private SparkMaxConfig rotationConfig;
private SparkMaxConfig algaeMotorConfig;
private SparkMaxConfig intakeConfig;
public DigitalInput coralLimitSwitch;
// private Encoder wristEncoder;
private double kP, kI, kD;
private double wristPos0 = 105;
// private DigitalInput EncoderLeft;
// private DigitalInput EncoderRight;



public Wrist(){
    intakeMotor = new SparkMax (Constants.WristConstants.intakeMotorPort, MotorType.kBrushless);
    algaeMotor = new SparkMax (Constants.WristConstants.algaeMotorPort, MotorType.kBrushless);
    rotationMotor = new SparkMax (Constants.WristConstants.wristRotationPort, MotorType.kBrushless);

    rotationConfig = new SparkMaxConfig();
    algaeMotorConfig= new SparkMaxConfig();
    intakeConfig=new SparkMaxConfig();

    // EncoderLeft = new DigitalInput(2);
    // EncoderRight = new DigitalInput(3);
    // wristEncoder = new Encoder(6, 7);
    coralLimitSwitch=new DigitalInput(Constants.WristConstants.limitSwitchPort);
    configureMotors();
    }
    
public void configureMotors(){
    // wrist rotation motor configuration 

    // SmartDashboard.putNumber("Wrist kP", Constants.WristConstants.wristRotationMotorkP);
    // SmartDashboard.putNumber("Wrist kI", Constants.WristConstants.wristRotationMotorkI);
    // SmartDashboard.putNumber("Wrist kD", Constants.WristConstants.wristRotationMotorkD);

    rotationConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake)
        .closedLoopRampRate(0.2);

    rotationConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360)
        .inverted(true)
        .zeroOffset(0.75);

    rotationConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(Constants.WristConstants.armWristP, Constants.WristConstants.armWristI , Constants.WristConstants.armWristD);
    
    rotationMotor.configure(rotationConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    algaeMotorConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    algaeMotor.configure(algaeMotorConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    intakeConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
    intakeMotor.configure(intakeConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

    }

@Override
    public void periodic() {
        SmartDashboard.putNumber("wrist pos", rotationMotor.getAbsoluteEncoder().getPosition());

        // this.kP = SmartDashboard.getNumber("Wrist kP", Constants.WristConstants.wristRotationMotorkP);
        // this.kI = SmartDashboard.getNumber("Wrist kI", Constants.WristConstants.wristRotationMotorkI);
        // this.kD = SmartDashboard.getNumber("Wrist kD", Constants.WristConstants.wristRotationMotorkD);
    }

public void setAngle(double wristAngle){

    rotationMotor.getClosedLoopController().setReference(wristAngle+WristConstants.writsRotationOffset, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    
    // SmartDashboard.putNumber("Wrist PID Output", rotationMotor.getAppliedOutput());
}

public void activateIntake(){
    // if(!coralLimitSwitch.get()){
    // intakeMotor.set(Constants.WristConstants.intakeSpeed);
    // }
    intakeMotor.set(Constants.WristConstants.intakeSpeed);
    spinAlgae();
}

public void spinAlgae(){
    algaeMotor.set(-Constants.WristConstants.algaeSpeed);
}

public void reverseIntake(){
    intakeMotor.set(-Constants.WristConstants.intakeSpeed);
}

public void reverseAlgae(){
    algaeMotor.set(Constants.WristConstants.algaeSpeed);
}

public void stopIntake(){
    // made so the scoring element motors will stop spinning. 
    intakeMotor.set(0);
    stopAlgae();
    // wristMotorAlgae.set(0);
}

public void stopAlgae(){
    algaeMotor.set(0);
}

public void manualRotate(){
    rotationMotor.set(0.5);
}

public void manualReverseRotate(){
    rotationMotor.set(-0.15);
}

public void stopRotation(){
    rotationMotor.set(0);
}
}
