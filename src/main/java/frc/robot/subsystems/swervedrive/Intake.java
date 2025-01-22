// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.swervedrive;

import com.ctre.phoenix6.StatusSignal;
import com.ctre.phoenix6.hardware.CANrange;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

import edu.wpi.first.util.sendable.Sendable;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  private SparkMax intakeMotor;
  private CANrange rangeSensor;

  public Intake() {
    intakeMotor = new SparkMax(3, MotorType.kBrushless);
    rangeSensor = new CANrange(5);
  }

  public void setSpeed(double speed){
    intakeMotor.set(speed);
  }

  public boolean hasCoral() {
    return rangeSensor.getIsDetected().getValue();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
    SmartDashboard.putBoolean("CANRange", hasCoral());
    SmartDashboard.putNumber("CANRangeDistance", rangeSensor.getDistance().getValueAsDouble());
  }

  public void off() {
    // TODO Auto-generated method stub
    throw new UnsupportedOperationException("Unimplemented method 'off'");
  }
}
