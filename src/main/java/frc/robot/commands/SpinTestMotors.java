package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class SpinTestMotors extends Command {

    Arm arm;
    public SpinTestMotors(Arm arm){
        this.arm=arm;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.spinTestMotors();
    }

    public boolean isFinished(){
        return false;
    }
}
