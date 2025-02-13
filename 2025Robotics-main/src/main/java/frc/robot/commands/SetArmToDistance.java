package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetArmToDistance extends Command {

    Arm arm;
    double desiredDistance;
    public SetArmToDistance(Arm arm, double desiredDistance){
        this.arm=arm;
        this.desiredDistance=desiredDistance;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.rotateWrist(desiredDistance);
    }

    public boolean isFinished(){
        if(Arm.extentionError<0.1){
            return true;
        }
        return false;
    }
}
