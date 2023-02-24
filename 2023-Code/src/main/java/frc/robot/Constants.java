// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

public final class Constants {
  public static class OperatorConstants {
    public static final int JOYSTICK_LEFT = 0;
    public static final int JOYSTICK_RIGHT = 1;
    public static final int XBOX = 2;
  }

  public static class PDPConstants {
    public static final int PDP_CAN_ID = 0;
  }

  public static class MotorConstants {
    // DRIVETRAIN
    public static final int MOTOR_RIGHT1 = 11;
    public static final int MOTOR_RIGHT2 = 12;
    public static final int MOTOR_LEFT1 = 13;
    public static final int MOTOR_LEFT2 = 14;

    // INTAKE
    public static final int INTAKEMOTOR = 21;

    // ARM
    public static final int ARM_MOTOR = 31;
  }

  public static class ArmConstants {
    public static final double UP = 56;
    public static final double FORWARD_DOWN = 90;
    public static final double FORWARD_MID = 80;
    public static final double BACK_MID = 31;
    public static final double BACK_DOWN = 18;
    public static final double ARM_DEADZONE = 1;

    public static final double ARM_BACKMOST = 17;
    public static final double ARM_FRONTMOST = 91;

    public static final double DEGREES_TO_ENCODER_VALUES = 74 / 275;

    public static final double GATHER_MODE_POSITION = 90;
    public static final double FEEDER_POSITION =
        FORWARD_MID + (14.47751 * DEGREES_TO_ENCODER_VALUES);
    public static final double INTAKE_RUNNABLE = 83;
  }

  public static class PIDConstants {
    public static final double dt = 20;

    public static final double ARM_P = 0.04;
    public static final int ARM_I = 0;
    public static final int ARM_D = 0;
    public static final int ARM_FF = 0;

    public static final double BALANCE_P = 0.08;
    public static final double BALANCE_I = 0;
    public static final double BALANCE_D = 0;

    public static final double MIN_OUTPUT = -.3;
    public static final double MAX_OUTPUT = .3;
  }

  public static class PneumaticsConstants {
    public static final int UP_DOWN_SOLENOID = 0;
    public static final int IN_OUT_SOLENOID = 1;
    public static final int GRABBER = 2;
  }

  public static class DrivetrainConstants {
    public static final double JOYSTICK_DEADZONE = 0.1;
    public static final double AUTO_BALANCE_DEADZONE = 0.4;
  }

  public static class DigitalConstants {
    public static final int INTAKE_BEAMBREAK = 0;
    public static final int GRABBER_BEAMBREAK = 1;
    public static final int INTAKE_DOWN_BEAMBREAK = 2;
    public static final int ARM_ENCODER = 9;
  }

  public static class PWMConstants {
    public static final int LED_PORT = 0;
  }

  public static class DriverConstants {
    public static final double GEAR_RATIO = 1.0 / 8.0;
    public static final double WHEEL_DIAMETER = 0.127;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double ENCODER_POSITION_CONVERSION = GEAR_RATIO * WHEEL_CIRCUMFERENCE;
    public static final double ENCODER_VELOCITY_CONVERSION = GEAR_RATIO * WHEEL_CIRCUMFERENCE / 60;
    public static final double MAXSPEED = 0.7;
  }

  public static class UtilConstants {
    public static final String UNIVERSAL_DIRECTORY =
        "\\src\\main\\java\\frc\\robot\\util\\trajectoryInput\\";
  }
}
