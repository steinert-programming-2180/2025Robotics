// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.AlgaeForward;
import frc.robot.commands.AlgaeReverse;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.commands.IntakeForward;
import frc.robot.commands.IntakeReverse;
import frc.robot.commands.SetArmToDistance;
import frc.robot.commands.SetBaseToAngle;
import frc.robot.commands.SetWristToAngle;
import frc.robot.subsystems.Arm;
import frc.robot.subsystems.ExampleSubsystem;
import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.InstantCommand;
// import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

public class RobotContainer {
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Arm arm = new Arm();

  private final AlgaeForward algaeForward=new AlgaeForward(arm);
  private final AlgaeReverse algaeReverse=new AlgaeReverse(arm);
  private final IntakeForward intakeForward=new IntakeForward(arm);
  private final IntakeReverse intakeReverse=new IntakeReverse(arm);

  //To do: make an enumerated list that lets desired angle/distance be altered without making new commands
  private final SetArmToDistance setArmToDistance=new SetArmToDistance(arm, 20);
  private final SetBaseToAngle SetBaseToAngle=new SetBaseToAngle(arm, 40);
  private final SetWristToAngle setWristToAngle=new SetWristToAngle(arm, 35);

  private final CommandPS5Controller PS5Controller=new CommandPS5Controller(OperatorConstants.PS5ControllerPort);  

  public RobotContainer() {

    configureBindings();
  }

  private void configureBindings() {
    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }

  
  private void pS5ConfigureButtonBindings() {

    //   m_ps5driverController.triangle().onTrue().onFalse();
    //   m_ps5driverController.cross().onTrue().onFalse();
  
    //   m_ps5driverController.R2().onTrue().onFalse();
  
    //   m_ps5driverController.circle().onTrue().onFalse();
    //   m_ps5driverController.square().onTrue().onFalse();
  
    //   m_ps5driverController.R1().onTrue().toggleOnFalse();
    //   m_ps5driverController.L1().onTrue().toggleOnFalse();
  
    //   m_ps5driverController.povUp().onTrue();
    //   m_ps5driverController.povDown().onTrue();
    
    //   m_driverController.povLeft().onTrue();
    //   m_driverController.povRight().onTrue().onFalse();
    //   //m_ps5driverController.povUp().onTrue();


}
}
