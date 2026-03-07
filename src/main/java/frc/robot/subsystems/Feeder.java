// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

import static frc.robot.Constants.FeederConstants.*;


public class Feeder extends SubsystemBase {

  @NotLogged
TalonFX upperMotor, lowerMotor;

@NotLogged private static Feeder singleton;


  /** Creates a new Feeder. */
  public Feeder() {
    upperMotor = new TalonFX(upperMotorID);
    lowerMotor = new TalonFX(lowerMotorID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = currentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    upperMotor.getConfigurator().apply(config);
    upperMotor.getVelocity().setUpdateFrequency(50); // Set update frequency to 50 Hert, 20ms
    upperMotor.getSupplyVoltage().setUpdateFrequency(20);
    upperMotor.getSupplyCurrent().setUpdateFrequency(20);
    upperMotor.getStatorCurrent().setUpdateFrequency(20);
    upperMotor.optimizeBusUtilization();

    config.MotorOutput.Inverted = InvertedValue.CounterClockwise_Positive;
    lowerMotor.getConfigurator().apply(config);
    lowerMotor.getVelocity().setUpdateFrequency(50); // Set update frequency to 50 Hert, 20ms
    lowerMotor.getSupplyVoltage().setUpdateFrequency(20);
    lowerMotor.getSupplyCurrent().setUpdateFrequency(20);
    lowerMotor.getStatorCurrent().setUpdateFrequency(20);
    lowerMotor.optimizeBusUtilization();

  }

  
    public void setSpeed (double speed) {
        upperMotor.setVoltage(speed);
        lowerMotor.setVoltage(speed);
    }

    public void stop () {
        setSpeed(0);
    }

  public static Feeder getInstance() {
    if (singleton == null)
      singleton = new Feeder();
    return singleton;
  }
    
    //Unjam method?

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
