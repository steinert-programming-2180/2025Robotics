package frc.robot.subsystems;

import com.revrobotics.RelativeEncoder;
import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkAbsoluteEncoder;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkClosedLoopController.ArbFFUnits;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.config.SparkFlexConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.ArmConstants;


public class Arm  extends SubsystemBase{
    private SparkMax armBaseMotor;
    private SparkMax armFollowMotor;
    private SparkMax armExtendMotor;
    private SparkMax armWristMotor;

    private double power;

    private SparkAbsoluteEncoder angleEncoder;

    // private SparkFlex intakeMotor;
    // private SparkFlex algaeMotor;

    private SparkMaxConfig armBaseConfig;
    private SparkMaxConfig armFollowConfig;

    private SparkMaxConfig armExtendConfig;
    // private SparkFlexConfig armWristConfig;

    DigitalInput bottomLimitSwitch=new DigitalInput(ArmConstants.bottomLimitSwitchID);

    public static double baseError;
    public static double extentionError;
    public static double wristError;

    public Arm() {
        armBaseMotor = new SparkMax(ArmConstants.armBaseMotorPort, MotorType.kBrushless);
        armFollowMotor = new SparkMax(ArmConstants.armFollowMotorPort, MotorType.kBrushless);

        armExtendMotor=new SparkMax(ArmConstants.armExtendMotorPort, MotorType.kBrushless);
        // armWristMotor=new SparkFlex(ArmConstants.armWristMotorPort, MotorType.kBrushless);

        // intakeMotor=new SparkFlex(ArmConstants.intakeMotorPort, MotorType.kBrushless);
        // algaeMotor=new SparkFlex(ArmConstants.algaeMotorPort, MotorType.kBrushless);

        armBaseConfig = new SparkMaxConfig();
        armFollowConfig = new SparkMaxConfig();

        armExtendConfig = new SparkMaxConfig();

        configMotors();
    }

    private void configMotors(){
        armBaseConfig
        .inverted(true)
        .idleMode(IdleMode.kBrake);
        // .closedLoopRampRate(0.1);

        armBaseConfig.absoluteEncoder
        .positionConversionFactor(360)
        .velocityConversionFactor(360)
        .zeroOffset(0.33);
        
        armBaseConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ArmConstants.armBaseP, ArmConstants.armBaseI, ArmConstants.armBaseD);

        armBaseMotor.configure(armBaseConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);


        
        // armFollowConfig.inverted(true);
        armFollowConfig.follow(armBaseMotor, false); // i'm the devil
        armFollowConfig.idleMode(IdleMode.kBrake);

        armFollowMotor.configure(armFollowConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        //The numbers here work with angles. Should be updated later to work with length
        armExtendConfig
        .inverted(false)
        .idleMode(IdleMode.kBrake);
        armExtendConfig.absoluteEncoder
        .positionConversionFactor(360)
        // multiply by Math.Pi*0.315 to mayb get distance
        .velocityConversionFactor(360);
        armExtendConfig.closedLoop
        .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        .pid(ArmConstants.armExtendP, ArmConstants.armExtendI, ArmConstants.armExtendD);

        armExtendMotor.configure(armExtendConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        // armWristConfig
        // .inverted(false)
        // .idleMode(IdleMode.kBrake);
        // armWristConfig.absoluteEncoder
        // .positionConversionFactor(360)
        // .velocityConversionFactor(360);
        // armWristConfig.closedLoop
        // .feedbackSensor(FeedbackSensor.kAbsoluteEncoder)
        // .pid(ArmConstants.armWristP, ArmConstants.armWristI, ArmConstants.armWristD);

        // armWristMotor.configure(armWristConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);

        angleEncoder = armBaseMotor.getAbsoluteEncoder();
    }

    // public void rotateBase(double desiredAngle){
    //     baseError = Math.abs(armBaseMotor.getAbsoluteEncoder().getPosition()-desiredAngle);
    
    //     if(baseError>=0.1){
    //     armBaseMotor.getClosedLoopController().setReference(desiredAngle, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    //     }
    // }

    @Override
    public void periodic() {
        SmartDashboard.putNumber("Arm Rotate Angle", getAngle());
        SmartDashboard.putNumber("Arm Rotate Error", getAngle() - armBaseMotor.getAbsoluteEncoder().getPosition());
        SmartDashboard.putBoolean("B Limit Switch", atBottomLimit());
        // SmartDashboard.putNumber("PID output", armBaseMotor.getAppliedOutput());

        // power=armBaseMotor.get();

        // if (!(leftBumper.getAsBoolean() || rightBumper.getAsBoolean())) {
        //     power = 0.0;
        // } else if (leftBumper.getAsBoolean()) {
        //     power = baseArmSpeed;
        // } else {
        //     power = -baseArmSpeed;
        // }
        
        if (atBottomLimit() && power > 0) {
            stopRotating();
        }

        // rotate(power);
    }

    public void extend(double desiredDistance){
        desiredDistance*=2*Math.PI*ArmConstants.armExtensionRadius;
        armWristMotor.getClosedLoopController().setReference(desiredDistance, ControlType.kPosition, ClosedLoopSlot.kSlot0);
    }

    public void rotate(double power) {

        if (Math.abs(power) < 0.01) {
            power = 0.0;
        }

        this.power = power;

        armBaseMotor.set(power);
    }

    public void setAngle(double angle) {

        armBaseMotor.getClosedLoopController().setReference(
        //Normal PID stuff
        ArmConstants.armAngleOffset - angle, ControlType.kPosition, ClosedLoopSlot.kSlot0, 
        //FeedForward :(
        Math.cos(ArmConstants.armAngleOffset - getAngle()) * 0.01, ArbFFUnits.kPercentOut);

        SmartDashboard.putNumber("Arm PID Output", armBaseMotor.get());
        

        // Math.sin(angle - getAngle()) / 4, ArbFFUnits.kPercentOut);
    }

    public double getAngle() {
        return angleEncoder.getPosition();
    }

    public void stopRotating() {
        // armBaseMotor.set(0.0);
        rotate(0.0);
    }

    public void retractArm(){
        armExtendMotor.set(-0.75);
    }

    public void extendArm(){
        armExtendMotor.set(0.75);
    }

    public void stopRetract() {
        armExtendMotor.set(0.0);
    }

    public boolean atBottomLimit(){
        return bottomLimitSwitch.get();
    }

    public void raiseArm(){
        // armBaseMotor.set(0.5);
        rotate(-0.5);
    }

    public void lowerArm() {
        // armBaseMotor.set(-0.3);
        rotate(0.75);
    }
}