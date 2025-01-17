package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SetWristToAngle extends Command {

    Arm arm;
    double desiredAngle;
    public SetWristToAngle(Arm arm, double desiredAngle){
        this.arm=arm;
        this.desiredAngle=desiredAngle;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.rotateWrist(desiredAngle);
    }

    public boolean isFinished(){
        if(Arm.wristError<0.1){
            return true;
        }
        return false;
    }
}
