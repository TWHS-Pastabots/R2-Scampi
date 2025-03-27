// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.RollerConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class to run the rollers over CAN */
public class CANRollerSubsystem extends SubsystemBase {
  private final SparkMax rollerMotor;
  public static CANRollerSubsystem instance;

  public CANRollerSubsystem() {
    // Set up the roller motor as a brushless motor
    rollerMotor = new SparkMax(RollerConstants.ROLLER_MOTOR_ID, MotorType.kBrushless);
    rollerMotor.setCANTimeout(250);

  
    SparkMaxConfig rollerConfig = new SparkMaxConfig();
    rollerConfig.voltageCompensation(RollerConstants.ROLLER_MOTOR_VOLTAGE_COMP);
    rollerConfig.smartCurrentLimit(RollerConstants.ROLLER_MOTOR_CURRENT_LIMIT);
    rollerMotor.configure(rollerConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public static CANRollerSubsystem getInstance(){
    if (instance == null){
      instance = new CANRollerSubsystem();
    }
    return instance;
  }

  /** This is a method that makes the roller spin */
  public void runRoller(double forward, double reverse) {
    rollerMotor.set(forward - reverse);
  }
  public void takeIn(){
    rollerMotor.set(-0.5);
  }
  public void reverseOut(){
    rollerMotor.set(0.5);
  }
  public void turnOff(){
    rollerMotor.set(0.0);
  }
 
}
