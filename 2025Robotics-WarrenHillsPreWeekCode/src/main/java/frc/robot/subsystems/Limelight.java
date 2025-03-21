package frc.robot.subsystems;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.LimelightHelpers;
import frc.robot.Constants.FieldConstants;
import frc.robot.Constants.LimelightConstants;


public class Limelight extends SubsystemBase {

  private NetworkTableEntry camMode;
  private NetworkTableEntry ledMode;
  private NetworkTableEntry tv;
  private NetworkTableEntry ty;
  private NetworkTableEntry tx;
  private NetworkTableEntry ta;
  private NetworkTableEntry tid;
  NetworkTable tableFront;
  NetworkTable tableBack;

  public Limelight() {
    // Initialize the Limelight here
    tableFront = NetworkTableInstance.getDefault().getTable("limelight-front");
    tableBack = NetworkTableInstance.getDefault().getTable("limelight-back");
    
    tx = tableFront.getEntry("tx");
    tx = tableBack.getEntry("tx");
    ty = tableFront.getEntry("ty");
    ty = tableBack.getEntry("ty");

    // Checks for target

    tv = tableFront.getEntry("tv");
    tv = tableBack.getEntry("tv");
    // Target area as a percentage of image
    ta = tableFront.getEntry("ta");
    ta = tableBack.getEntry("ta");

    tid = tableFront.getEntry("tid");
    tid = tableBack.getEntry("tid");

    camMode = tableFront.getEntry("camMode");
    camMode = tableBack.getEntry("camMode");
    ledMode = tableFront.getEntry("ledMode");
    ledMode = tableBack.getEntry("ledMode");

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
    return LimelightHelpers.getBotPose2d(getName());
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
    SmartDashboard.putString("AprilTag Pose", getAprilTag());
   }
}