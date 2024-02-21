// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;


import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkLowLevel.MotorType;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.ArmFeedforward;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.lib.k;

public class AmpSubsystem extends SubsystemBase {
  // Define and initialize the two spin motors
  TalonFX m_leftSpinMotor = new TalonFX(k.RIO_CAN_BUS_IDS.AMP_LEFT_SPIN_MOTOR);
  TalonFX m_rightSpinMotor = new TalonFX(k.RIO_CAN_BUS_IDS.AMP_RIGHT_SPIN_MOTOR);
  // Create a VoltageOut instance that the TalonFX needs to set a voltage
  VoltageOut m_spinVoltage = new VoltageOut(0);

  // Create a rotate motor instance of the CANSparkMax
  CANSparkMax m_rotateMotor = new CANSparkMax(k.RIO_CAN_BUS_IDS.AMP_ROTATE_MOTOR, MotorType.kBrushless);

  // These constraints are used to limit the max velocity and acceleration of the rotate motor
  // The actual control of the motor is in Volts, so the PID needs to return volts. Therefore Velocity is in Volts*radians/sec
  TrapezoidProfile.Constraints m_rotateConstraints = new TrapezoidProfile.Constraints(
    k.AMP.ROTATE_MOTOR_MAX_SPEED_RAD_PER_SEC*k.AMP.ROTATE_MOTOR_VELOCITY_SCALE, 
    k.AMP.ROTATE_MOTOR_MAX_SPEED_RAD_PER_SEC*k.AMP.ROTATE_MOTOR_VELOCITY_SCALE /10.0);
  // This is a PID controller that uses the constraints to limit the velocity and acceleration
  ProfiledPIDController m_rotateProPID = new ProfiledPIDController(0, 0, 0, m_rotateConstraints);
  // A arm feedforward uses a Cos term to adjust for gravity.
  // ks is in Volts, kg in Volts, kv in volt seconds per radian, ka in  volt seconds² per radian.
  ArmFeedforward m_rotateArmFeedForward = new ArmFeedforward(0, 0, k.AMP.ROTATE_MOTOR_FF_KV, k.AMP.ROTATE_MOTOR_FF_KA);
  
  /**
   * Spin the intake wheels at a set speed of +/- 1.0
   * @param _speed speed of +/- 1.0
   */
  public void spin(double _speed){
    double spinVoltage = _speed * k.ROBOT.MAX_BATTERY_VOLTAGE;

    m_leftSpinMotor.setControl(m_spinVoltage.withEnableFOC(true).withOutput(spinVoltage));
    m_rightSpinMotor.setControl(m_spinVoltage.withEnableFOC(true).withOutput(spinVoltage));
  }
  /** Rotate the motor to some degrees using a ProfiledPID and a ArmFeedForward
   * 
   * @param _angle +/- 180 degrees
   */
  public void rotate(double _angle){
    // Calculate the ProfiledPID value based on the actual angle and desired angle
    double pid = m_rotateProPID.calculate(Math.toRadians(getActualAngle()), Math.toRadians(_angle));
    // Calculate the Feedforward term based on the angle to adjust for gravity and the PID desired velocity
    double ff = m_rotateArmFeedForward.calculate(Math.toRadians(getActualAngle()), m_rotateProPID.getSetpoint().velocity);
    
    // Clamp the PID value to limit its output until we have control of the device
    pid = MathUtil.clamp(pid, -1, 1);
    // SmartDashboard way of suppling a voltage to the motor for calibration
    if(SmartDashboard.getBoolean("AMP Test Volt Enable", false)){
      double volts = SmartDashboard.getNumber("AMP_TestVolts", 0.0);
      m_rotateMotor.setVoltage(volts);
    }else {
      // If button is not pressed set the value to 0.
      m_rotateMotor.setVoltage(0);
      // Actual control of motor with PID and Feedforward. Commented out until calibration done.
      //m_rotateMotor.setVoltage(pid + ff);
    }
  
  }

  public double getActualAngle(){
    return m_rotateMotor.getEncoder().getPosition() / k.AMP.ROTATE_MOTOR_GEAR_RATIO * 360.0;
  }
  /** Creates a new AmpSubsystem. */
  public AmpSubsystem() {
    initialize();;
  }
  public void initialize(){
    SmartDashboard.putNumber("AMP_TestVolts", 0.0);
    SmartDashboard.putBoolean("AMP Test Volt Enable", false);
  }
  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
