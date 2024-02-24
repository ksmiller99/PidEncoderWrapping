/**
 * +090 degrees is straight up
 *  000 degrees is horizontal forward
 * +180 degrees is horizontal backwards
 * 
 * > +180 never legal
 * <  000 never legal
 */
package frc.robot.subsystems;

import static com.revrobotics.SparkLimitSwitch.Type.kNormallyOpen;
import static frc.robot.Constants.ArmConstants.initialAllowedError;
import static frc.robot.Constants.ArmConstants.initialD;
import static frc.robot.Constants.ArmConstants.initialFF;
import static frc.robot.Constants.ArmConstants.initialI;
import static frc.robot.Constants.ArmConstants.initialIz;
import static frc.robot.Constants.ArmConstants.initialMaxAcc;
import static frc.robot.Constants.ArmConstants.initialMaxAngle;
import static frc.robot.Constants.ArmConstants.initialMaxOutput;
import static frc.robot.Constants.ArmConstants.initialMaxRPM;
import static frc.robot.Constants.ArmConstants.initialMaxVel;
import static frc.robot.Constants.ArmConstants.initialMinAngle;
import static frc.robot.Constants.ArmConstants.initialMinOutput;
import static frc.robot.Constants.ArmConstants.initialMinVel;
import static frc.robot.Constants.ArmConstants.initialP;

import com.revrobotics.CANSparkBase.IdleMode;
import com.revrobotics.CANSparkLowLevel.MotorType;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkAbsoluteEncoder;
import com.revrobotics.SparkLimitSwitch;
import com.revrobotics.SparkPIDController;
import com.revrobotics.SparkAbsoluteEncoder.Type;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class SmartMotionArm extends SubsystemBase {
  private CANSparkMax leadMotor; // right side
  private CANSparkMax followMotor; //  left side
  private SparkPIDController pidController;
  private SparkAbsoluteEncoder m_encoder; // through-bore connected to follow SparkMax
  private SparkLimitSwitch m_forwardLimit;
  private SparkLimitSwitch m_reverseLimit;
  public double kP, kI, kD, kIz, kFF, maxOutput, minOutput, maxRPM, maxVel, minVel, maxAcc, allowedErr, maxAngle,
      minAngle;
  private double angleGoal;
  private double startingAngle = 0; // straight up
  private boolean kEncoderInverted = true;

  //use degrees for position
  private static final double kEncoderPositionFactor = 360; // degrees
  private static final double kEncoderVelocityFactor = 360 / 60; // degrees/second

  public double getAngleGoal() {
    return angleGoal;
  }

  /*
   * Set the position goal in angle, >= 0
   */
  public void setAngleGoal(double angle) {
    System.out.println("setAngleGoal:" + angle);
    if (!IsAngleGoalValid(angle))
      return;
    angleGoal = angle;
    pidController.setReference(angle, CANSparkMax.ControlType.kSmartMotion);
  }

  public boolean IsAngleGoalValid(double angle) {
    if (angle < minAngle || angle > maxAngle) {
      System.out.println("Illegal Arm Angle Goal: " + angle);
      return false;
    }
    return true;
  }

  public SmartMotionArm() {
    leadMotor = new CANSparkMax(Constants.CANIDs.kArmMotorRight, MotorType.kBrushless);
    leadMotor.restoreFactoryDefaults();
    leadMotor.setInverted(false);
    leadMotor.setIdleMode(IdleMode.kCoast);

    m_forwardLimit = leadMotor.getForwardLimitSwitch(kNormallyOpen);
    m_reverseLimit = leadMotor.getReverseLimitSwitch(kNormallyOpen);

    followMotor = new CANSparkMax(Constants.CANIDs.kArmMotorLeft, MotorType.kBrushless);
    followMotor.restoreFactoryDefaults();
    followMotor.follow(leadMotor, true);
    
    // initialze PID controller and encoder objects
    pidController = leadMotor.getPIDController();
    m_encoder = leadMotor.getAbsoluteEncoder(Type.kDutyCycle);
  
    m_encoder.setPositionConversionFactor(kEncoderPositionFactor);
    m_encoder.setVelocityConversionFactor(kEncoderVelocityFactor);
    m_encoder.setInverted(kEncoderInverted);
    
    System.out.println("Z : "+m_encoder.getZeroOffset());
    m_encoder.setZeroOffset(0);
    System.out.println("P : "+m_encoder.getPosition());
    m_encoder.setZeroOffset(30.0);
    System.out.println("Z2: "+m_encoder.getZeroOffset());
    
    // PID coefficients
    kP = initialP;
    kI = initialI;
    kD = initialD;
    kIz = initialIz;
    kFF = initialFF;
    maxOutput = initialMaxOutput;
    minOutput = initialMinOutput;
    maxRPM = initialMaxRPM;

    // Smart Motion Coefficients
    maxVel = initialMaxVel; // rpm
    minVel = initialMinVel; // rpm
    maxAcc = initialMaxAcc;
    allowedErr = initialAllowedError;
    maxAngle = initialMaxAngle;
    minAngle = initialMinAngle;

    pidController.setPositionPIDWrappingEnabled(true);
    pidController.setPositionPIDWrappingMinInput(0);
    pidController.setPositionPIDWrappingMaxInput(180);

    // set PID coefficients
    pidController.setP(kP);
    pidController.setI(kI);
    pidController.setD(kD);
    pidController.setIZone(kIz);
    pidController.setFF(kFF);
    pidController.setOutputRange(minOutput, maxOutput);

    int smartMotionSlot = 0;
    pidController.setFeedbackDevice(m_encoder);
    pidController.setSmartMotionMaxVelocity(maxVel, smartMotionSlot);
    pidController.setSmartMotionMinOutputVelocity(minVel, smartMotionSlot);
    pidController.setSmartMotionMaxAccel(maxAcc, smartMotionSlot);
    pidController.setSmartMotionAllowedClosedLoopError(allowedErr, smartMotionSlot);

    setAngleGoal(startingAngle);
  }

  @Override
  public void periodic() {
    SmartDashboard.putNumber("angleSeen", m_encoder.getPosition());
    SmartDashboard.putNumber("angleWanted", getAngleGoal());
  }

  public void Stop() {
    leadMotor.stopMotor();
  }

  /**
   * Get the current status of the Forward and Reverse limit switches
   * 
   * @return boolean[]{Fwd, Rev}
   */
  public boolean[] getLimitSwitches() {
    return new boolean[] { m_forwardLimit.isPressed(), m_reverseLimit.isPressed()};
  }

}

