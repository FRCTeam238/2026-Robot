// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BLinePath;
import frc.robot.commands.CalcLaunchSequence;
import frc.robot.commands.IntakeMid;
import frc.robot.commands.LaunchSequence;
import frc.robot.commands.SnapToHub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class OneLineBumpTest extends SequentialCommandGroup {
  /** Creates a new OneLineBumpTest. */
  public OneLineBumpTest() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BLinePath("singlePathMidAuto", false, true),
      new SnapToHub().withTimeout(1),
      new ProxyCommand(new LaunchSequence(48).deadlineFor(new WaitCommand(3).andThen(new IntakeMid())).withTimeout(9)),
      new BLinePath("singlePathFinalBump", false, false)
      );
  }
}
