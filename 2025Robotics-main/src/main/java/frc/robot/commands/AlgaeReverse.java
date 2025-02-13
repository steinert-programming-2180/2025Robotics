package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class AlgaeReverse extends Command {

    Arm arm;
    public AlgaeReverse(Arm arm){
        this.arm=arm;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.reverseAlgae();
    }

    public boolean isFinished(){
        return false;
    }
}
