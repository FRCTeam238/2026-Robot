// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Drivetrain;

/* You should consider using the more terse Command factories API instead https://docs.wpilib.org/en/stable/docs/software/commandbased/organizing-command-based.html#defining-commands */
public class XLock extends Command {
  /** Creates a new XLock. */
  public XLock() {
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(Drivetrain.getInstance());
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    
    Drivetrain.getInstance().setCommand("XLock");
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Drivetrain.getInstance().lockWheels();

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    Drivetrain.getInstance().setCommand("");

  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
