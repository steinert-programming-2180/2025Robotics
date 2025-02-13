package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class IntakeReverse extends Command {

    Arm arm;
    public IntakeReverse(Arm arm){
        this.arm=arm;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.reverseIntake();
    }

    public boolean isFinished(){
        return false;
    }
}
