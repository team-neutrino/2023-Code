// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class OperatorConstants {
    public static final int JOYSTICK_LEFT = 1;
    public static final int JOYSTICK_RIGHT = 0;
    public static final int XBOX = 4;
  }

  public static class PDPConstants {
    public static final int PDP_CAN_ID = 0;
  }

  public static class MotorConstants {
    public static final int INTAKEMOTOR1 = 21; // intake motors have '2' in front of actual id

    public static final int MOTOR_RIGHT1 = 3;
    public static final int MOTOR_RIGHT2 = 4;
    public static final int MOTOR_LEFT1 = 1;
    public static final int MOTOR_LEFT2 = 2;

    // ARM
    public static final int ARM_MOTOR1 = 31; // replace
  }

  public static class ArmConstants {
    public static final double UP = 56;
    public static final double FORWARD_DOWN = 90;
    public static final double FORWARD_MID = 80;
    public static final double BACK_MID = 31;
    public static final double BACK_DOWN = 17;

    public static final double M_PI = Math.PI;
    public static final double WHEEL_SIZE = 1;
    public static final double REDUCTION = 1;
    public static final double ROTATION_TO_INCHES = M_PI * (WHEEL_SIZE / REDUCTION);
    public static final float MIN_SOFT_LIM = -100;
    public static final float MAX_SOFT_LIM = 100;
    public static final int ARM_DOWN = 0;
    public static final int ARM_UP = 90;
  }

  public static class PIDConstants {
    public static final double dt = 20;

    public static final double ARM_P = 0.07;
    public static final int ARM_I = 0;
    public static final int ARM_D = 0;
    public static final int ARM_FF = 0;
    public static final int ARM_MINIMUM = -1;
    public static final int ARM_MAXIMUM = 1;

    public static final double BALANCE_P = 0.2;
    public static final double BALANCE_I = 0;
    public static final double BALANCE_D = 0;
  }

  public static class PneumaticsConstants {
    public static final int UP_DOWN_SOLENOID = 0;
    public static final int IN_OUT_SOLENOID = 1;
    public static final int GRABBER = 2;
  }

  public static class VariableConstants {
    public static final double DEADZONE = 0.1;
  }

  public static class DigitalConstants {
    public static final int INDEX_BEAMBREAK = 0;
    public static final int GRABBER_BEAMBREAK = 1;
    public static final int ARM_ENCODER = 4;
  }

  public static class DriverConstants {
    public static final double GEAR_RATIO = 1.0 / 8.0;
    public static final double WHEEL_DIAMETER = 0.127;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double ENCODER_POSITION_CONVERSION = GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    public static final double ENCODER_VELOCITY_CONVERSION = GEAR_RATIO * WHEEL_CIRCUMFERENCE / 60;
  }
}
