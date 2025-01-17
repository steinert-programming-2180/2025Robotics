package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetBaseToAngle extends Command {

    Arm arm;
    double desiredAngle;
    public SetBaseToAngle(Arm arm, double desiredAngle){
        this.arm=arm;
        this.desiredAngle=desiredAngle;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.rotateBase(desiredAngle);
    }

    public boolean isFinished(){
        if(Arm.baseError<0.1){
            return true;
        }
        return false;
    }
}
