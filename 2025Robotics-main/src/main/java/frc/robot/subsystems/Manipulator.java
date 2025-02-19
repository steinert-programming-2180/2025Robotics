package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Manipulator extends SubsystemBase {

    private Arm m_arm;
    private Wrist m_wrist;
    
    // Constructor
    public Manipulator(Arm m_arm, Wrist m_wrist) {
        
        this.m_arm = m_arm;
        this.m_wrist = m_wrist;
    }

    // Method to set State
    public void setState() {

    }

}
