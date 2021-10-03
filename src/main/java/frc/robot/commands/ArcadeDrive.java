// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.Constants;
import frc.robot.Constants.JoystickAxis;
import frc.robot.subsystems.DriveTrain;

public class ArcadeDrive extends CommandBase {

  private final DriveTrain _drivetrain;
  private final Joystick _joystick;

  /** Creates a new ArcadeDrive. */
  public ArcadeDrive(DriveTrain dt, Joystick joy) {
    // Use addRequirements() here to declare subsystem dependencies.
    _drivetrain = dt;
    _joystick = joy;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    _drivetrain.arcadeDrive(-0.8 * _joystick.getRawAxis(Constants.JoystickAxis.YAxis),
                             0.8 * _joystick.getRawAxis(Constants.JoystickAxis.XAxis));

    

  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return false;
  }
}
