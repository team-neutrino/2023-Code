package frc.robot.util;

import frc.robot.Constants;
import frc.robot.Constants.PIDConstants;

public class ViennaPIDController {

  private double m_p;
  private double m_i;
  private double m_d;
  private double m_f;

  private double m_iState;
  private double m_iZone;

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

  public void setIZone(double p_iZone) {
    m_iZone = p_iZone;
  }

  private double updateI(double error, double currentState) {
    if(Math.abs(error) <= m_iZone || m_iZone == Constants.PIDConstants.IZONE_DISABLE_KEY) {
      currentState += error * PIDConstants.dt;
    } else {
      currentState = 0;
    }
    return currentState;
  }

  private double bounder(double unbounded) {
    return Math.min(Math.max(unbounded, Constants.PIDConstants.MIN_OUTPUT), Constants.PIDConstants.MAX_OUTPUT);
  }

  public double run(double realPos, double desiredPos) {
    double error = desiredPos - realPos;

    /*Integral calculation */
    m_iState = updateI(error, m_iState);

    /*Derivative calculation */
    double derivative = (error - previousError) / PIDConstants.dt;
    previousError = error;

    double ff = desiredPos * m_f;

    double output = m_p * error + m_i * m_iState + m_d * derivative + ff;

    return bounder(output);
  }
}
