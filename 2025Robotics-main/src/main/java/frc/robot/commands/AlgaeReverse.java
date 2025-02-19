package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;

import frc.robot.subsystems.Wrist;

public class AlgaeReverse extends Command {

    Wrist m_Wrist; 
    
    public AlgaeReverse(Wrist m_Wrist){
        this.m_Wrist = m_Wrist;
    }

    public void initialize(){
        
    }

    public void execute(){
        m_Wrist.wristMotorBackwardAlgae();
    }

    public boolean isFinished(){
        // return !m_Wrist.stopIntakeForBeamBreak();
        return false;
    }
}
