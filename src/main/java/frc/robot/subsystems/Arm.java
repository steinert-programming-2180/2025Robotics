package frc.robot.subsystems;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import frc.robot.Constants.ArmConstants;
import frc.robot.Constants.PhysicalConstants;
import frc.robot.Constants.testConstants;


public class Arm {
    private SparkFlex armBaseMotor;
    private SparkFlex armExtendMotor;
    private SparkFlex armWristMotor;

    private SparkFlex intakeMotor;
    private SparkFlex algaeMotor;

    private SparkFlex testMotor1;
    private SparkFlex testMotor2;

    private SparkFlexConfig armBaseConfig;
    private SparkFlexConfig armExtendConfig;
    private SparkFlexConfig armWristConfig;

    public static double baseError;
    public static double extentionError;
    public static double wristError;

    public Arm (){
        armBaseMotor=new SparkFlex(ArmConstants.armBaseMotorPort, MotorType.kBrushless);
        armExtendMotor=new SparkFlex(ArmConstants.armExtendMotorPort, MotorType.kBrushless);
        armWristMotor=new SparkFlex(ArmConstants.armWristMotorPort, MotorType.kBrushless);

        testMotor1=new SparkFlex(testConstants.motor1Id, MotorType.kBrushless);
        testMotor2=new SparkFlex(testConstants.motor2Id, MotorType.kBrushless);

        intakeMotor=new SparkFlex(ArmConstants.intakeMotorPort, MotorType.kBrushless);
        algaeMotor=new SparkFlex(ArmConstants.algaeMotorPort, MotorType.kBrushless);

        armBaseConfig=new SparkFlexConfig();

        configMotors();
    }

    private void configMotors(){
        armBaseConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        armBaseConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360);
        armBaseConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ArmConstants.armBaseP, ArmConstants.armBaseI, ArmConstants.armBaseD);

        armBaseMotor.configure(armBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //The numbers here work with angles. Should be updated later to work with length
        armExtendConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        armExtendConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360);
        armExtendConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ArmConstants.armExtendP, ArmConstants.armExtendI, ArmConstants.armExtendD);

        armExtendMotor.configure(armExtendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        armWristConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        armWristConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360);
        armWristConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ArmConstants.armWristP, ArmConstants.armWristI, ArmConstants.armWristD);

        armWristMotor.configure(armWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    }

    public void rotateBase(double desiredAngle){
        baseError=Math.abs(armBaseMotor.getAbsoluteEncoder().getPosition()-desiredAngle);
        
        if(baseError>=0.1){
        armBaseMotor.getClosedLoopController().setReference(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

    //The numbers here work with angles. Should be updated later to work with length
    public void extendArm(double desiredDistance){
        extentionError=Math.abs(armExtendMotor.getAbsoluteEncoder().getPosition()-desiredDistance);

        if(extentionError>=0.1){
        armWristMotor.getClosedLoopController().setReference(desiredDistance, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }
    
    public void rotateWrist(double desiredAngle){
        wristError=Math.abs(armExtendMotor.getAbsoluteEncoder().getPosition()-desiredAngle);

        if(wristError>=0.1){
        armWristMotor.getClosedLoopController().setReference(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
        }
    }

    public void spinIntake(){
        intakeMotor.set(ArmConstants.intakeSpeed);
    }

    public void spinAlgae(){
        algaeMotor.set(ArmConstants.algaeSpeed);
    }

    public void reverseIntake(){
        intakeMotor.set(-ArmConstants.intakeSpeed);
    }

    public void reverseAlgae(){
        algaeMotor.set(-ArmConstants.algaeSpeed);
    }

    public void spinTestMotors(){
        testMotor1.set(0.8);
        testMotor2.set(0.8);
    }

    public void stopTestMotors(){
        testMotor1.set(0);
        testMotor2.set(0);
    }
}
