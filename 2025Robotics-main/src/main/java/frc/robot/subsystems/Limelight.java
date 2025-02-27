package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.LimelightConstants;
import frc.robot.Robot;
import frc.robot.Constants.FieldConstants;
import edu.wpi.first.wpilibj.Servo;


public class Limelight extends SubsystemBase {

  private NetworkTableEntry camMode;
  private NetworkTableEntry ledMode;
  private NetworkTableEntry tv;
  private NetworkTableEntry ty;
  private NetworkTableEntry tx;
  private NetworkTableEntry ta;
  private NetworkTableEntry tid;
  public Servo limeServo;
  private int servoCounter;
  private int prevCounter;
  NetworkTable table;

  public Limelight() {
    // Initialize the Limelight here
    table = NetworkTableInstance.getDefault().getTable("limelightNetworkTable");
    
    tx = table.getEntry("tx");
    ty = table.getEntry("ty");

    // Checks for target
    tv = table.getEntry("tv");
    // Target area as a percentage of image
    ta = table.getEntry("ta");

    tid = table.getEntry("tid");

    camMode = table.getEntry("camMode");
    ledMode = table.getEntry("ledMode");

    prevCounter=0;
    servoCounter=0;

  }

  public double getStraightDistance() {
    if (!hasTarget()) {
            return -1;
        }

        // Gets our angle from horizontal by adding angle of limelight to angle offset
        double theta = Math.toRadians(ty.getDouble(0) + LimelightConstants.LIME_ANGLE);

        // theta = 0 implies sin(theta) = 0, so to avoid division
        // by zero, return -1 if theta = 0
        if (theta == 0) {
            return -1;
        }

        // Gets delta height from top of target to where the limelight is mounted
        double height = Constants.inchesToMeters(FieldConstants.TARGET_MAX_HEIGHT - LimelightConstants.LIME_HEIGHT);

        // Returns distance by using trigonometry
        return height / Math.sin(theta);
  }

  public Pose2d getPose() {
    return LimelightHelpers.getBotPose2d("Starscream,");
  }

  public double getCameraMode() {
    return (double) camMode.getNumber(0);
  }

  public double getLightsMode() {
    return (double) ledMode.getNumber(0);
  }

  public boolean isTracking() {
    return (getCameraMode() == 0) && (getLightsMode() == 0);
  }

  public boolean hasTarget() {
    return (isTracking() && tv.getDouble(0) == 1);
  }

  public double getTargetAngle() {
    if (!hasTarget()) {
            return -1;
        }
        return tx.getDouble(0);
   }

   public double getArea() {
    if (!hasTarget()) {
            return -1;
        }
        return ta.getDouble(0);
   }

   public Number getApriltag(){
    return tid.getNumber(0);
   }

   public String getAprilTag(){
      switch("ktag" + getApriltag().intValue()){
        case "ktag1": return "Red Loading Station 1";
        case "ktag2": return "Red Loading Station 2";
        case "ktag3": return "Blue Processing Station 1";
        case "ktag4": return "Blue Barge 1";
        case "ktag5": return "Red Barge 1";
        case "ktag6": return "Red Reef 1";
        case "ktag7": return "Red Reef 2";
        case "ktag8": return "Red Reef 3";
        case "ktag9": return "Red Reef 4";
        case "ktag10": return "Red Reef 5";
        case "ktag11": return "Red Reef 6";
        case "ktag12": return "Blue Loading Station 1";
        case "ktag13": return "Blue Loading Station 2";
        case "ktag14": return "Blue Barge 2";
        case "ktag15": return "Red Barge 2";
        case "ktag16": return "Red Processing Station 1";
        case "ktag17": return "Blue Reef 1";
        case "ktag18": return "Blue Reef 2";
        case "ktag19": return "Blue Reef 3";
        case "ktag20": return "Blue Reef 4";
        case "ktag21": return "Blue Reef 5";
        case "ktag22": return "Blue Reef 6";
        default: return "No Tag Found";
      }
   }

   public void publishToSmartdash() {
    SmartDashboard.putNumber("tx", tx.getDouble(0));
    SmartDashboard.putNumber("ty", ty.getDouble(0));

    SmartDashboard.putNumber("tv", tv.getDouble(0));
    SmartDashboard.putNumber("ta", ta.getDouble(0));
    SmartDashboard.putString("Which AprilTag?", getAprilTag());
    SmartDashboard.putNumber("counter", servoCounter);
   }

   public void topTilt() {
    limeServo.setAngle(LimelightConstants.LIME_TILTUP);
   }

   public void bottomTilt() {
    limeServo.setAngle(LimelightConstants.LIME_TILTDOWN);
   }

   public void neutralTilt() {
    limeServo.setAngle(LimelightConstants.LIME_TILTNEUTRAL);
   }
   public double getTiltAngle() {
    return limeServo.getAngle();
   }

   public void incrementServo(){
    prevCounter=servoCounter;
    servoCounter++;


    if(servoCounter%4==0){
        bottomTilt();
      }
      else if(servoCounter%4==1){
        neutralTilt();
      }
      else if(servoCounter%4==2){
        topTilt();
      }
      else if(servoCounter%4==3){
        neutralTilt();
      }
   }

   public int getServoCounter(){
    return servoCounter;
   }
   public int getPrevCounter(){
    return prevCounter;
   }
}

