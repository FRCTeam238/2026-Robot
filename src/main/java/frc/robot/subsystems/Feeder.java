// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FeederConstants.*;


public class Feeder extends SubsystemBase {

TalonFX leftMotor, rightMotor;


  /** Creates a new Feeder. */
  public Feeder() {
    leftMotor = new TalonFX(leftMotorID);
    rightMotor = new TalonFX(rightMotorID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    leftMotor.getConfigurator().apply(config);
    leftMotor.getVelocity().setUpdateFrequency(50); // Set update frequency to 50 Hert, 20ms
    leftMotor.getSupplyVoltage().setUpdateFrequency(20);
    leftMotor.getSupplyCurrent().setUpdateFrequency(20);
    leftMotor.getStatorCurrent().setUpdateFrequency(20);
    leftMotor.optimizeBusUtilization();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    rightMotor.getConfigurator().apply(config);
    rightMotor.getVelocity().setUpdateFrequency(50); // Set update frequency to 50 Hert, 20ms
    rightMotor.getSupplyVoltage().setUpdateFrequency(20);
    rightMotor.getSupplyCurrent().setUpdateFrequency(20);
    rightMotor.getStatorCurrent().setUpdateFrequency(20);
    rightMotor.optimizeBusUtilization();

  }

  
    public void setSpeed (double speed) {
        leftMotor.setVoltage(speed);
        rightMotor.setVoltage(speed);
    }

    public void stop () {
        setSpeed(0);
    }
    
    //Unjam method?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
