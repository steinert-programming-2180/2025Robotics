package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.Wrist;

public class SetWristToAngle extends Command {

    Wrist m_Wrist;
    double m_wristAngle;
    public SetWristToAngle(Wrist m_Wrist, double m_wristAngle){
        this.m_Wrist = m_Wrist;
        this.m_wristAngle = m_wristAngle;
    }

    public void initialize(){
        
    }

    public void execute(){
        m_Wrist.rotateTheWrist(m_wristAngle);
    }

    public boolean isFinished(){
        if(Wrist.howMuchToPos<=0.1){
            return true;
        } //Hi judges :)
        return false;
    }
}
