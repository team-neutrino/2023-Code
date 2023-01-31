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
    public static final int JOYSTICK_RIGHT = 0;
    public static final int JOYSTICK_LEFT = 1;
    public static final int XBOX = 4;
  }

  public static class PDPConstants {
    public static final int PDP_CAN_ID = 0;
  }

  public static class MotorConstants {
    public static final int INTAKEMOTOR1 = 21; // intake motors have '2' in front of actual id
    public static final int INTAKEMOTOR2 = 22;
    public static final int INTAKEMOTOR3 = 23; // not used currently, may be in the future

    public static final int RMOTOR1 = 3;
    public static final int RMOTOR2 = 4;
    // switch these back later
    public static final int LMOTOR1 = 1;
    public static final int LMOTOR2 = 2;

    public static final int ARM_MOTOR1 = 31;
    public static final int GRABBER_MOTOR1 = 41;
    public static final int GRABBER_MOTOR2 = 42;
  }

  public static class ArmConstants {
    public static final double M_PI = Math.PI;
    public static final double WHEEL_SIZE = 1;
    public static final double REDUCTION = 1;
    public static final double ROTATION_TO_INCHES = M_PI * (WHEEL_SIZE / REDUCTION);
    public static final float MIN_SOFT_LIM = -100;
    public static final float MAX_SOFT_LIM = 100;
  }

  public static class PIDConstants {
    public static final double ARM_P = 0.2;
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
    public static final int INTAKE1 = 0;
    public static final int INTAKE2 = 1;
    public static final int GRABBER = 2;
    public static final int SOLENOID_FRONT = 3;
    public static final int SOLENOID_BACK = 4;
  }

  public static class VariableConstants {
    public static final double DEADZONE = 0.1;
  }

  public static class DigitalConstants {
    public static final int INDEX_BEAMBREAK = 0;
  }

  public static class DriverContants {
    public static final double GEAR_RATIO = 1.0/8.0;
    public static final double WHEEL_DIAMETER = 0.127;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double ENCODER_CONVERSION = GEAR_RATIO * WHEEL_CIRCUMFERENCE;
  }
}
