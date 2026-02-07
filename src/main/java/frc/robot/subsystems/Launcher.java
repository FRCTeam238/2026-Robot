// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.Follower;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.MotorAlignmentValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.LauncherConstants.*;

@Logged
public class Launcher extends SubsystemBase {

  TalonFX leftUp, leftLow, rightUp, rightLow;
  double requestedSpeed=0;

  @NotLogged private static Launcher singleton;

  /** Creates a new Launcher. */
  public Launcher() {
    leftUp = new TalonFX(leftUpID);
    leftLow = new TalonFX(leftLowID);
    rightUp = new TalonFX(rightUpID);
    rightLow = new TalonFX(rightLowID);

     var config = new TalonFXConfiguration();
    config.Slot0.kP = kP;
    config.Slot0.kI = kI;
    config.Slot0.kD = kD;
    config.Slot0.kV = kV;
    config.Slot0.kS = kS;
    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    leftUp.getConfigurator().apply(config);
    leftUp.getVelocity().setUpdateFrequency(50); // Set update frequency to 50 Hert, 20ms
    leftUp.getClosedLoopError().setUpdateFrequency(50);
    leftUp.getClosedLoopOutput().setUpdateFrequency(50);
    leftUp.getSupplyVoltage().setUpdateFrequency(20);
    leftUp.getSupplyCurrent().setUpdateFrequency(20);
    leftUp.getStatorCurrent().setUpdateFrequency(20);
    leftUp.optimizeBusUtilization();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    rightUp.getConfigurator().apply(config);
    rightUp.getVelocity().setUpdateFrequency(50); // Set update frequency to 50 Hert, 20ms
    rightUp.getClosedLoopError().setUpdateFrequency(50);
    rightUp.getClosedLoopOutput().setUpdateFrequency(50);
    rightUp.getSupplyVoltage().setUpdateFrequency(20);
    rightUp.getSupplyCurrent().setUpdateFrequency(20);
    rightUp.getStatorCurrent().setUpdateFrequency(20);
    rightUp.optimizeBusUtilization();

    leftLow.setControl(new Follower(leftUp.getDeviceID(), MotorAlignmentValue.Aligned));
    rightLow.setControl(new Follower(rightUp.getDeviceID(), MotorAlignmentValue.Aligned));
  }

  public void setSpeed(double speed) {
    requestedSpeed = speed;
    leftUp.setControl(new VelocityVoltage(speed));
    rightUp.setControl(new VelocityVoltage(speed));
  }

  public void stop() {
        setSpeed(0);
  }

  public boolean atSpeed() {
    double leftError = leftUp.getClosedLoopError().getValueAsDouble();
    if(Math.abs(leftError / requestedSpeed * 100) > tolerance)
    {
      return false;
    }
    double rightError = rightUp.getClosedLoopError().getValueAsDouble();
    if(Math.abs(rightError / requestedSpeed * 100) > tolerance)
    {   
      return false;
    }
    return true;
  }

  public static Launcher getInstance() {
    if (singleton == null)
      singleton = new Launcher();
    return singleton;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
