package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Wrist;

public class IntakeReverse extends Command {

    Wrist m_Wrist;
    public IntakeReverse(Wrist m_Wrist){
        this.m_Wrist = m_Wrist;
    }

    public void initialize(){
        
    }

    public void execute(){
        m_Wrist.wristIntakeBackwardCoral();
    }

    public boolean isFinished(){
        return m_Wrist.getBeamBreak();
    }
}
