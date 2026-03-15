// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import frc.robot.subsystems.Intake;

import static frc.robot.Constants.IntakeConstants.intakeMid;
import static frc.robot.Constants.IntakeConstants.intakeRollerVoltage;

import edu.wpi.first.wpilibj2.command.Command;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class IntakeMid extends Command {
  /** Creates a new IntakeMid. */
  public IntakeMid() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Intake.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    Intake.getInstance().setTiltPosition(intakeMid);
    Intake.getInstance().setCommand("IntakeMid");
  }
  
  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {

    //if(Intake.getInstance().tiltAtTarget()) {     //Will spin the intake rollers indefinetely in this command?
    Intake.getInstance().runIntake(intakeRollerVoltage);
    //  } 
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Intake.getInstance().setCommand("");
    Intake.getInstance().stopRoller();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return Intake.getInstance().tiltAtTarget();
  }
}
