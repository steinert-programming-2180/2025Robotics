package frc.robot.commands;



import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.robot.RobotContainer;
import frc.robot.Constants.OperatorConstants;



public class ps5Rumble extends Command{
    
    private CommandPS5Controller m_DriverPS5Controller;

    public ps5Rumble(){
        this.m_DriverPS5Controller = RobotContainer.m_ps5driverController;
    }


    public void execute(){
        if(DriverStation.getMatchTime() == 10){
            m_DriverPS5Controller.setRumble(GenericHID.RumbleType.kBothRumble, OperatorConstants.PS5ControllerRumble);
        }
    }

    public boolean isFinished(){
        if(DriverStation.getMatchTime() == 11){
            return true; 
        }
        return false;
    }
}

    