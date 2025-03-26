package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;

/** Class to run the rollers over CAN */
public class CANPivotSubsystem extends SubsystemBase {
  private final SparkMax pivotMotor;
  public static CANPivotSubsystem instance;

  public CANPivotSubsystem() {
    // Set up the roller motor as a brushless motor
    pivotMotor = new SparkMax(PivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotMotor.setCANTimeout(250);

  
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.voltageCompensation(PivotConstants.PIVOT_MOTOR_VOLTAGE_COMP);
    pivotConfig.smartCurrentLimit(PivotConstants.PIVOT_MOTOR_CURRENT_LIMIT);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
  }

  @Override
  public void periodic() {
  }

  /** This is a method that makes the roller spin */
  public static CANPivotSubsystem getInstance(){
    if (instance == null){
      instance = new CANPivotSubsystem();
    }
    return instance;
  }

  /** This is a method that makes the roller spin */
  public void runPivot(double forward, double reverse) {
    pivotMotor.set(forward - reverse);
  }
  public void moveDown(){
    pivotMotor.set(1.0);
  }
  public void moveUp(){
    pivotMotor.set(-1.0);
  }
  public void turnOff(){
    pivotMotor.set(0.0);
  }
 
}

