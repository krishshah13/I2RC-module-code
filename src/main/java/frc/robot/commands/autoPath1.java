// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.RobotContainer;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class autoPath1 extends SequentialCommandGroup {
  /** Creates a new autoPath1. */
  private RobotContainer m_robotcontainer = new RobotContainer();

  public autoPath1() {
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new DriveForward(m_robotcontainer.getDriveTrain(), 6.9), new TurnToAngle(m_robotcontainer.getDriveTrain(), 90));
  }
}