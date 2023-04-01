// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
 * TO DO:
 * - driver backward controller to a lower button on the joystick
 * - turn telescoping into a diff subsystem
 * - be able to retract telescoping while using armToAngle and arm adjust
 * - feeder lower (check all the angles)
 * -
 */
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
    public static final double INTAKE_MOTOR_SPEED = .7;

    // ARM
    public static final int ARM_MOTOR = 31;
    public static final int TELESCOPING_MOTOR = 32;
  }

  public static class ArmConstants {
    //Xbox abxy default positions
    public static final double FORWARD_DOWN = 94;
    public static final double FORWARD_MID = 80.5;
    public static final double BACK_MID = 36.7;
    public static final double BACK_DOWN = 18;

    public static final double FEEDER = 34.5;

    public static final double BACK_HIGH_CONE = 37.7;

    public static final double BACK_MID_CUBE = 30;
    public static final double BACK_HIGH_CUBE = 34;

    public static final double ARM_DEADZONE = 1;
    public static final double CONE_ADDITION = 3;

    public static final double ARM_BACKMOST = 18;
    public static final double ARM_FRONTMOST = 95;

    public static final double GATHER_POSITION = 91;
    public static final double INTAKE_RUNNABLE = 84;

    public static final double ARM_OUTPUT_LIMIT = 0.2;
    public static final double SCALE_FACTOR = 5;
    public static final double ARM_INPUT_DEADZONE = 0.5;

    public static final double FORWARD_ARM_HEIGHT_LIMIT = 73; // actual limit 69
    public static final double BACKWARD_ARM_HEIGHT_LIMIT = 40; // actual limit 44
    public static final double FORWARD_FLOOR_ARM_ANGLE_LIMIT = 90;
  }

  public static class TelescopeConstants {

    public static final double TELESCOPE_EXTEND_SPEED = 0.5; // note: setInverted is true
    public static final double TELESCOPE_RETRACT_SPEED = -0.5;
    public static final double TELESCOPE_EXTEND_LIMIT = -5500;
    public static final double TELESCOPING_DEADZONE = 250;

    public static final double TELESCOPE_HEIGHT_LIMIT_RETRACT_SPEED = -0.85;
  }

  public static class PIDConstants {
    public static final double dt = 20;
    public static final double ARM_P = 0.06;
    public static final double ARM_P_ADJUST = 0.02;
    public static final double ARM_EXTENSION_P = .02;
    public static final double ARM_I = 0;
    public static final double ARM_D = 0;
    public static final double ARM_FF = .01;

    public static final double BALANCE_P = 0.1;
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
    public static final double AUTO_BALANCE_DEADZONE = .4;
    public static final double BALANCE_MOTOR_POWER = -.3;
  }

  public static class DigitalConstants {
    public static final int INTAKE_BEAMBREAK = 0;
    public static final int GRABBER_BEAMBREAK = 4;
    public static final int INTAKE_DOWN_BEAMBREAK = 2;
    public static final int TELESCOPING_ENCODERA = 7;
    public static final int TELESCOPING_ENCODERB = 8;
    public static final int ARM_ENCODER = 9;
  }

  public static class PWMConstants {
    public static final int LED_PORT = 2;
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
