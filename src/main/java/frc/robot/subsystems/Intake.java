// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix6.SignalLogger;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.epilogue.NotLogged;
import edu.wpi.first.units.measure.Voltage;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine;

import static edu.wpi.first.units.Units.Second;
import static edu.wpi.first.units.Units.Volts;
import static frc.robot.Constants.IntakeConstants.*;

@Logged
public class Intake extends SubsystemBase {
  /** Creates a new Intake. */
  @NotLogged
  TalonFX rollerMotor, tiltMotor;
  @NotLogged
  CANcoder tiltSensor;
  double targetPosition = 0;
  private String command = "";

  @NotLogged
  private static Intake singleton;

  public Intake() {

    tiltSensor = new CANcoder(tiltID);
    tiltMotor = new TalonFX(tiltID);
    rollerMotor = new TalonFX(rollerID);

    var config = new TalonFXConfiguration();
    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive;
    config.MotorOutput.NeutralMode = NeutralModeValue.Coast;
    config.CurrentLimits.StatorCurrentLimit = rollerStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = rollerSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    rollerMotor.getConfigurator().apply(config);
    rollerMotor.getVelocity().setUpdateFrequency(20);
    rollerMotor.getSupplyVoltage().setUpdateFrequency(20);
    rollerMotor.getSupplyCurrent().setUpdateFrequency(20);
    rollerMotor.getStatorCurrent().setUpdateFrequency(20);
    rollerMotor.optimizeBusUtilization();

    config.MotorOutput.Inverted = InvertedValue.Clockwise_Positive; // Positive value stows intake
    config.MotorOutput.NeutralMode = NeutralModeValue.Brake;
    config.CurrentLimits.StatorCurrentLimit = tiltStatorCurrentLimit;
    config.CurrentLimits.StatorCurrentLimitEnable = true;
    config.CurrentLimits.SupplyCurrentLimit = tiltSupplyCurrentLimit;
    config.CurrentLimits.SupplyCurrentLimitEnable = true;
    config.Feedback.withRemoteCANcoder(tiltSensor);
    config.Feedback.RotorToSensorRatio = intakePivotRatio;

    // set slot 0 gains
    var slot0Configs = config.Slot0;
    slot0Configs.kS = tiltKS;
    slot0Configs.kV = tiltKV;
    slot0Configs.kA = tiltKA;
    slot0Configs.kP = tiltKP;
    slot0Configs.kI = tiltKI;
    slot0Configs.kD = tiltKD;
    slot0Configs.GravityType = GravityTypeValue.Arm_Cosine;
    slot0Configs.kG = tiltKG;

    // set Motion Magic Expo settings
    var motionMagicConfigs = config.MotionMagic;
    motionMagicConfigs.MotionMagicCruiseVelocity = tiltCruise;
    motionMagicConfigs.MotionMagicAcceleration = tiltAcceleration;
    motionMagicConfigs.MotionMagicJerk = tiltJerk;
    // motionMagicConfigs.MotionMagicExpo_kV = tiltExpoKV;
    // motionMagicConfigs.MotionMagicExpo_kA = tiltExpoKA;

    config.SoftwareLimitSwitch.ForwardSoftLimitThreshold = intakeUp;
    config.SoftwareLimitSwitch.ForwardSoftLimitEnable = true;
    config.SoftwareLimitSwitch.ReverseSoftLimitThreshold = intakeDown;
    config.SoftwareLimitSwitch.ReverseSoftLimitEnable = true;

    tiltMotor.getConfigurator().apply(config);

    tiltMotor.getConfigurator().apply(config);
    tiltMotor.getVelocity().setUpdateFrequency(50); // Set update frequency to 50 Hertz, 20ms
    tiltMotor.getPosition().setUpdateFrequency(50);
    tiltMotor.getClosedLoopError().setUpdateFrequency(50);
    tiltMotor.getClosedLoopOutput().setUpdateFrequency(50);
    tiltMotor.getClosedLoopReference().setUpdateFrequency(50);
    tiltMotor.getSupplyVoltage().setUpdateFrequency(20);
    tiltMotor.getSupplyCurrent().setUpdateFrequency(20);
    tiltMotor.getStatorCurrent().setUpdateFrequency(20);
    tiltMotor.optimizeBusUtilization();
  }

  public void setCommand(String name) {
    command = name;
  }

  public void setTiltPosition(double position) {
    targetPosition = position;
    tiltMotor.setControl(new MotionMagicVoltage(position));
  }

  public void stopTilt() {
    tiltMotor.stopMotor();
  }

  public double getTargetPosition() {
    return targetPosition;
  }

  public void runIntake(double voltage) {
    rollerMotor.setVoltage(voltage);
  }

  public boolean tiltAtTarget() {
    double currentPosition = tiltMotor.getPosition().getValueAsDouble();
    double error = Math.abs(currentPosition - targetPosition);

    return error < tiltTolerance;
  }

  public void stopRoller() {
    runIntake(0);
  }

  public static Intake getInstance() {
    if (singleton == null)
      singleton = new Intake();
    return singleton;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

    private final SysIdRoutine m_sysIdRoutine =
   new SysIdRoutine(
      new SysIdRoutine.Config(
         Volts.of(0.1).per(Second),        // Use default ramp rate (1 V/s)
         Volts.of(1), // Reduce dynamic step voltage to 4 to prevent brownout
         null,        // Use default timeout (10 s)
                      // Log state with Phoenix SignalLogger class
         (state) -> SignalLogger.writeString("state", state.toString())
      ),
      new SysIdRoutine.Mechanism(
         (volts) -> tiltMotor.setControl(new VelocityVoltage((volts.in(Volts)))),
         null,
         this
      )
   );

  public Command sysIdQuasistatic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.quasistatic(direction);
  }

  public Command sysIdDynamic(SysIdRoutine.Direction direction) {
   return m_sysIdRoutine.dynamic(direction);
  }

}
