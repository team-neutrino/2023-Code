package frc.robot.util;

import frc.robot.Constants.PIDConstants;
import frc.robot.Constants.TelescopeConstants;

public class ViennaPIDController {

  private double m_p;
  private double m_i;
  private double m_d;
  private double m_f;

  private double m_iState;

  private double previousError = 0;

  public ViennaPIDController() {}

  public ViennaPIDController(double p_p) {
    m_p = p_p;
    m_i = 0;
    m_d = 0;
    m_f = 0;
  }

  public ViennaPIDController(double p_p, double p_i) {
    m_p = p_p;
    m_i = p_i;
    m_d = 0;
    m_f = 0;
  }

  public ViennaPIDController(double p_p, double p_i, double p_d) {
    m_p = p_p;
    m_i = p_i;
    m_d = p_d;
    m_f = 0;
  }

  public ViennaPIDController(double p_p, double p_i, double p_d, double p_f) {
    m_p = p_p;
    m_i = p_i;
    m_d = p_d;
    m_f = p_f;
  }

  public double getP() {
    return m_p;
  }

  public double getI() {
    return m_i;
  }

  public double getD() {
    return m_d;
  }

  public double getF() {
    return m_f;
  }

  public void setP(double p_p) {
    m_p = p_p;
  }

  public void setI(double p_i) {
    m_i = p_i;
  }

  public void setD(double p_d) {
    m_d = p_d;
  }

  public double run(double realPos, double desiredPos) {
    double error = desiredPos - realPos;

    /*Integral calculation */
    m_iState += error * PIDConstants.dt;

    /*Derivative calculation */
    double derivative = (error - previousError) / PIDConstants.dt;
    previousError = error;

    double ff = desiredPos * m_f;

    double output = m_p * error + m_i * m_iState + m_d * derivative + ff;

    return Limiter.bound(output, PIDConstants.MIN_OUTPUT, PIDConstants.MAX_OUTPUT);
  }

  public double run(double realPos, double desiredPos, double zone) {
    double error = desiredPos - realPos;
    error = Limiter.deadzone(error, zone);

    /*Integral calculation */
    m_iState += error * PIDConstants.dt;

    /*Derivative calculation */
    double derivative = (error - previousError) / PIDConstants.dt;
    previousError = error;

    double ff = desiredPos * m_f;

    double output = m_p * error + m_i * m_iState + m_d * derivative + ff;

    return Limiter.bound(output, PIDConstants.MIN_OUTPUT, PIDConstants.MAX_OUTPUT);
  }

  public double armRun(double realPos, double desiredPos, double armExtension) {
    double error = desiredPos - realPos;

    /*Integral calculation */
    m_iState += error * PIDConstants.dt;

    /*Derivative calculation */
    double derivative = (error - previousError) / PIDConstants.dt;
    previousError = error;

    double ff = desiredPos * m_f;

    double pAlteration =
        PIDConstants.ARM_EXTENSION_P * armExtension / TelescopeConstants.TELESCOPE_EXTEND_LIMIT;

    double output = (pAlteration + m_p) * error + m_i * m_iState + m_d * derivative + ff;

    return Limiter.bound(output, PIDConstants.MIN_OUTPUT, PIDConstants.MAX_OUTPUT);
  }
}
