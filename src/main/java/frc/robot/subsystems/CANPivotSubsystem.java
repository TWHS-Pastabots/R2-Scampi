package frc.robot.subsystems;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.PivotConstants;
import frc.robot.Constants.PivotConstants;

import com.revrobotics.spark.ClosedLoopSlot;
import com.revrobotics.spark.SparkClosedLoopController;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkBase.ControlType;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/** Class to run the rollers over CAN */
public class CANPivotSubsystem extends SubsystemBase {
  public final SparkMax pivotMotor;
  public static CANPivotSubsystem instance;
  public SparkClosedLoopController PID;
  public ArmFeedforward feedforward;
  public enum pivotStates{
    Algae(5.23),
    Base(-.54),
    Coral(.6);

    

    public double pose;
    
    private pivotStates(double postion) {
      pose = postion;
    }
  }
   public pivotStates pivotstates = pivotStates.Base;
  

  public CANPivotSubsystem() {
    // Set up the roller motor as a brushless motor
    pivotMotor = new SparkMax(PivotConstants.PIVOT_MOTOR_ID, MotorType.kBrushless);
    pivotMotor.setCANTimeout(250);

  
    SparkMaxConfig pivotConfig = new SparkMaxConfig();
    pivotConfig.closedLoop
       .feedbackSensor(FeedbackSensor.kPrimaryEncoder).pid(.07, 0, 0)
       .outputRange(-1, 1);
    //pivotConfig.closedLoopRampRate(.6);
    pivotConfig.idleMode(IdleMode.kBrake);
    pivotConfig.voltageCompensation(PivotConstants.PIVOT_MOTOR_VOLTAGE_COMP);
    pivotConfig.smartCurrentLimit(PivotConstants.PIVOT_MOTOR_CURRENT_LIMIT);
    pivotMotor.configure(pivotConfig, ResetMode.kResetSafeParameters, PersistMode.kPersistParameters);
    PID = pivotMotor.getClosedLoopController();
    feedforward = new ArmFeedforward(0, .5, 0);
  }


   public void updatepose(){
    PID.setReference(pivotstates.pose, ControlType.kPosition, ClosedLoopSlot.kSlot0,
   -(feedforward.calculate(-.261799*pivotMotor.getEncoder().getPosition()+1.41372, 0)));
   }

   public void pivotSetState(pivotStates tape){
    pivotstates = tape;
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
  // public void runPivot(double forward, double reverse) {
  //   pivotMotor.set(forward - reverse);
  // }
  public void moveDown(double speed){
    pivotMotor.set(speed);
  }
  public void moveUp(double speed){
    pivotMotor.set(-speed);
  }
  public void turnOff(){
    pivotMotor.set(0.0);
  }
 
}

