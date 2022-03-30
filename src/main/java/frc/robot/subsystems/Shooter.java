// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.RelativeEncoder;
import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.SparkMaxPIDController.ArbFFUnits;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.ShooterConstants;
import io.github.oblarg.oblog.Loggable;
import io.github.oblarg.oblog.annotations.Config;
import io.github.oblarg.oblog.annotations.Log;

public class Shooter extends SubsystemBase implements Loggable {
  CANSparkMax frontFlywheel, rearFlywheel;
  SparkMaxPIDController frontController, rearController;
  RelativeEncoder frontEncoder, rearEncoder;
  @Log
  private double frontSpeed = 1000;
  @Log
  private double rearSpeed = 1000;

  public Shooter() {
    frontFlywheel = new CANSparkMax(1, MotorType.kBrushless);
    rearFlywheel = new CANSparkMax(2, MotorType.kBrushless);
    frontController = frontFlywheel.getPIDController();
    rearController = rearFlywheel.getPIDController();
    frontEncoder = frontFlywheel.getEncoder();
    rearEncoder = rearFlywheel.getEncoder();
  }



  @Log
  private boolean atSetpoint() {
    return Math.abs(frontEncoder.getVelocity() - frontSpeed) < ShooterConstants.flywheelSpeedToleranceRPM && Math.abs(rearEncoder.getVelocity() - rearSpeed) < ShooterConstants.flywheelSpeedToleranceRPM;
  }

  @Config
  public void setFrontFlywheelSpeed(double velocity) {
    frontSpeed = velocity;
    frontController.setReference(frontSpeed, ControlType.kVelocity);//, 0, flywheelFeedforward.calculate(frontSpeed), ArbFFUnits.kVoltage);
  }

  @Config
  public void setRearFlywheelSpeed(double velocity) {
    rearSpeed = velocity;
    rearController.setReference(rearSpeed, ControlType.kVelocity);//, 0, flywheelFeedforward.calculate(rearSpeed), ArbFFUnits.kVoltage);
  }

  @Log
  public double getRealFrontSpeed() {
    return frontEncoder.getVelocity();
  }

  @Log
  public double frontP() {
    return frontController.getP();
  }

  @Log
  public double rearP() {
    return rearController.getP();
  }

  @Log
  public double frontF() {
    return frontController.getFF();
  }

  @Log
  public double rearF() {
    return rearController.getFF();
  }

  @Log
  public double getRealRearSpeed() {
    return rearEncoder.getVelocity();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
