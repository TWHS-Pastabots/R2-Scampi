// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.hal.FRCNetComm.tResourceType;
import edu.wpi.first.hal.HAL;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.commands.DriveCommand;
import frc.robot.subsystems.CANDriveSubsystem;
import frc.robot.subsystems.CANPivotSubsystem;
import frc.robot.subsystems.CANRollerSubsystem;
import frc.robot.subsystems.CANPivotSubsystem.pivotStates;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the
 * name of this class or
 * the package after creating this project, you must also update the
 * build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  private Command m_autonomousCommand;
  private CANPivotSubsystem pivot;
  private CANRollerSubsystem roller;
  private CANDriveSubsystem drive;
  private DriveCommand driveCommand;

  private static  XboxController driver;
  private static XboxController operator;

  private boolean wasXPresed;

  

  /**
   * This function is run when the robot is first started up and should be used
   * for any
   * initialization code.
   */
  @Override
  public void robotInit() {
    roller  = CANRollerSubsystem.getInstance();
    drive = CANDriveSubsystem.getInstance();
    pivot = CANPivotSubsystem.getInstance();
    driver = new XboxController(0);
    operator = new XboxController(1);

    boolean wasXPressed = false;
    
    // Instantiate our RobotContainer. This will perform all our button bindings,
    // and put our
    // autonomous chooser on the dashboard.

    // Used to track usage of the KitBot code, please do not remove
    HAL.report(tResourceType.kResourceType_Framework, 9);
  }

  /**
   * This function is called every 20 ms, no matter the mode. Use this for items
   * like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>
   * This runs after the mode specific periodic functions, but before LiveWindow
   * and
   * SmartDashboard integrated updating.
   */
  @Override
  public void robotPeriodic() {
<<<<<<< HEAD
    // pivot.updatepose();
=======
    SmartDashboard.putNumber("encoder pMotor val", pivot.pivotMotor.getEncoder().getPosition());

  
>>>>>>> 4e32a6375a786896d8986ce5d9e03067d57fcae7



    CommandScheduler.getInstance().run();
  }

  /** This function is called once each time the robot enters Disabled mode. */
  @Override
  public void disabledInit() {
  }

  @Override
  public void disabledPeriodic() {
  }

  /**
   * This autonomous runs the autonomous command selected by your
   * {@link RobotContainer} class.
   */
  @Override
  public void autonomousInit() {
    // schedule the autonomous command (example)
    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    double time = Timer.getFPGATimestamp();
    if(time < 100){
      drive.driveArcade(-.3, 0);
    }
    else {
      drive.driveArcade(0,0);
    }

  if(time > 20){
    roller.takeIn();
  }
  }

  @Override
  public void teleopInit() {
    // This makes sure that the autonomous stops running when
    // teleop starts running. If you want the autonomous to
    // continue until interrupted by another command, remove
    // this line or comment it out.
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
    
  }

  /** This function is called periodically during operator control. */
  @Override
  public void teleopPeriodic() {
<<<<<<< HEAD
   drive.driveArcade(driver.getLeftY(), driver.getRightX());
=======
    
    pivot.updatepose();
    
    
    if(operator.getLeftBumperButton())
      pivot.pivotSetState(pivotStates.Algae);
    


   drive.driveArcade(-driver.getLeftY(), -driver.getRightX());
>>>>>>> 4e32a6375a786896d8986ce5d9e03067d57fcae7
   
if(operator.getBButton()){
  roller.algae();
}
else
roller.turnOff();
   
   if(operator.getLeftTriggerAxis()> 0.5){
    roller.takeIn();
   }else if(operator.getRightTriggerAxis()>0.5){
    roller.reverseOut();
   }else{
   roller.turnOff();
   }

 
   if(operator.getYButton()){
<<<<<<< HEAD
    pivot.moveDown(0.2);
   }else if(operator.getAButton()){
    pivot.moveUp(0.2);
   }else{
    pivot.turnOff(0);
   }
=======
    pivot.pivotSetState(pivotStates.Coral);
   }else if(operator.getAButton()){
    pivot.pivotSetState(pivotStates.Algae);
   }else if(operator.getPOV() == 0){
    pivot.pivotSetState(pivotStates.Base);
   }

>>>>>>> 4e32a6375a786896d8986ce5d9e03067d57fcae7
   }


  

  @Override
  public void testInit() {
    // Cancels all running commands at the start of test mode.
    CommandScheduler.getInstance().cancelAll();
  }

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {
  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {
  }

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {
  }
}
