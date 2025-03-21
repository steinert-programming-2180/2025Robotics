package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {

    private Arm arm;
    private Wrist wrist;
    
    // Constructor
    public Manipulator(Arm arm, Wrist wrist) {
        
        this.arm = arm;
        this.wrist = wrist;
    }
    

    //all of these values need to be tuned btw
    public void reefLvl1(){
        arm.setAngle(20);
        arm.extend(10);
        wrist.setAngle(20);
    }

    public void reefLvl2(){
        arm.setAngle(30);
        arm.extend(14);
        wrist.setAngle(25);
    }
    
    public void reefLvl3(){
        arm.setAngle(40);
        arm.extend(17);
        wrist.setAngle(30);
    }

    public void reefLvl4(){
        arm.setAngle(50);
        // arm.extend(20);
        // wrist.setAngle(35);
    }

    public void humanPlayer(){
        arm.setAngle(37.5);
        arm.extend(12.5);
        wrist.setAngle(40);
    }

    // Method to set State
    public void setState() {
        
    }

}
