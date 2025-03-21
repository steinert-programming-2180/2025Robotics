package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class RotateArm extends Command {

    private Arm arm;

    public RotateArm(Arm arm){
        this.arm = arm;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.rotate(0.5);
    }

    public boolean isFinished(){
        if(Arm.baseError<=0.1){
            return true;
        }
        return false;
    }

    public void end(boolean isInterrupted) {
        arm.stopRotating();
    }
}
