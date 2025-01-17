package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class IntakeForward extends Command {

    Arm arm;
    public IntakeForward(Arm arm){
        this.arm=arm;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.spinIntake();
    }

    public boolean isFinished(){
        return false;
    }
}
