// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.autos;

import edu.wpi.first.wpilibj2.command.ProxyCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.BLinePath;
import frc.robot.commands.CalcLaunchSequence;
import frc.robot.commands.IntakeFuel;
import frc.robot.commands.IntakeMid;
import frc.robot.commands.SnapToHub;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class HideAndSeek_CaseysVersion extends SequentialCommandGroup {
  /** Creates a new HideAndSeek_CaseysVersion. */
  public HideAndSeek_CaseysVersion(boolean rightSide) {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(
      new BLinePath("Hide", rightSide, true),//2.22 Bline time. ~+.4 = ~2.75
      //Could probably change based on alliance members??
      new IntakeFuel().withTimeout(1.25),                    //~4
      new BLinePath("AndSeek_CaseysVersion", rightSide, false),
      new WaitCommand(1.5),
      new BLinePath("GOTCHA_CaseysVersion", rightSide, false),
      new SnapToHub().withTimeout(1),
      new ProxyCommand(new CalcLaunchSequence().deadlineFor(new WaitCommand(1).andThen(new IntakeMid()).andThen(new IntakeFuel())))
    );
  }
}
