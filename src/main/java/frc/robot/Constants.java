// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide
 * numerical or boolean
 * constants. This class should not be used for any other purpose. All constants
 * should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>
 * It is advised to statically import this class (or one of its inner classes)
 * wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class DriveConstants {
    public static final int LEFT_LEADER_ID = 6;
   public static final int LEFT_FOLLOWER_ID = 10;
   public static final int RIGHT_LEADER_ID = 8;
   public static final int RIGHT_FOLLOWER_ID = 5;

    public static final int DRIVE_MOTOR_CURRENT_LIMIT = 60;
  }

  public static final class RollerConstants {
   public static final int ROLLER_MOTOR_ID = 7;
    public static final int ROLLER_MOTOR_CURRENT_LIMIT = 20;
    public static final double ROLLER_MOTOR_VOLTAGE_COMP = 10;
    public static final double ROLLER_EJECT_VALUE = 0.44;
  }

  public static final class OperatorConstants {
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
  }
  public static final class PivotConstants {
    public static final int PIVOT_MOTOR_ID = 9;
    public static final int PIVOT_MOTOR_CURRENT_LIMIT = 40;
    public static final double PIVOT_MOTOR_VOLTAGE_COMP = 10;
    public static final double PIVOT_MOVE_VALUE = 0.44;
  }
}
