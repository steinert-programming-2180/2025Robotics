package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class AlgaeForward extends Command {

    Arm arm;
    public AlgaeForward(Arm arm){
        this.arm=arm;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.spinAlgae();
    }

    public boolean isFinished(){
        return false;
    }
}
