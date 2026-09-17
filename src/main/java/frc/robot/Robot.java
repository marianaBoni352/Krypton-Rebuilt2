// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

    private static final int kIntakeRackMotorId = 17;
    private static final int kIntakeRollerMotorId = 1;
    private static final double kIntakeSpeed = 0.5;


  private static final XboxController p1Controller = new XboxController(1);


  private final RobotContainer m_robotContainer;
  
    public TalonFX intakeRackMotor;
    public TalonFX intakeRollereMotor;
    
        private TalonFX kIntakeRackMotor;
        private TalonFX kIntakeRollerMotor;
      
        public Robot() {
          m_robotContainer = new RobotContainer();
          kIntakeRackMotor = new TalonFX(kIntakeRackMotorId);
          kIntakeRollerMotor = new TalonFX(kIntakeRollerMotorId);
  
      TalonFXConfiguration config = new TalonFXConfiguration();
  
      kIntakeRackMotor.getConfigurator().apply(config);
      kIntakeRollerMotor.getConfigurator().apply(config);
  
    }
  
    @Override
    public void robotPeriodic() {
      CommandScheduler.getInstance().run();
    }
   
    
    @Override
    public void disabledInit() {}
  
    @Override
    public void disabledPeriodic() {}
  
    @Override
    public void disabledExit() {}
  
    @Override
    public void autonomousInit() {   
      m_autonomousCommand = m_robotContainer.getAutonomousCommand();
  
      if (m_autonomousCommand != null) {
        CommandScheduler.getInstance().schedule(m_autonomousCommand);
      }
    }
  
    @Override
    public void autonomousPeriodic() {}
  
    @Override
    public void autonomousExit() {}
  
    @Override
    public void teleopInit() {
      if (m_autonomousCommand != null) {
        m_autonomousCommand.cancel();
      }
    }
  
    @Override
    public void teleopPeriodic() {
        if (p1Controller.getAButton()) {
        kIntakeRackMotor.set(kIntakeSpeed);
    } else if (p1Controller.getBButton()) {
        kIntakeRackMotor.set(-kIntakeSpeed);
    }  else {
        kIntakeRackMotor.stopMotor();
    }
        if (p1Controller.getAButton()) {
        kIntakeRackMotor.set(kIntakeSpeed);
    } else if (p1Controller.getBButton()) {
        kIntakeRackMotor.set(-kIntakeSpeed);
    }  else {
        kIntakeRackMotor.stopMotor();
    }
  } 
  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}
}
