// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import static frc.robot.Constants.IntakeConstants.intakeDown;
import static frc.robot.Constants.IntakeConstants.intakeVoltage;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Intake;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeFuel extends Command {
  /** Creates a new Intake. */
  public IntakeFuel() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(Intake.getInstance().getTargetPosition() == intakeDown && Intake.getInstance().tiltAtTarget())
    {
      Intake.getInstance().runIntake(intakeVoltage);
    } else
    { 
      Intake.getInstance().stopRoller();
    }
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.getInstance().stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
