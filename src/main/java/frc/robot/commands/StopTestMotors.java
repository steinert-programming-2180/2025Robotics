package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;

public class StopTestMotors extends Command {

    Arm arm;
    public StopTestMotors(Arm arm){
        this.arm=arm;
    }

    public void initialize(){
        
    }

    public void execute(){
        arm.stopTestMotors();
    }

    public boolean isFinished(){
        return false;
    }
}
