package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class AlgaeForward extends Command {

    Wrist m_Wrist;
    
    public AlgaeForward(Wrist m_Wrist){
        this.m_Wrist = m_Wrist;
    }

    public void initialize(){
        
    }

    public void execute(){
       m_Wrist.wristMotorForwardAlgae();
    }

    public boolean isFinished(){
        // return m_Wrist.stopIntakeForBeamBreak();
        return false;
    }
}
