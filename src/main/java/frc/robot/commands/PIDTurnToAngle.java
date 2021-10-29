// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class PIDTurnToAngle extends CommandBase {
  private final DriveTrain _driveTrain;
  private double angle;
  private int constant = 1;
  private double error;
  private double speed;

  /** Creates a new TimedAuto. */
  public PIDTurnToAngle(DriveTrain dt, double newAngle) {
    _driveTrain = dt;
    angle = newAngle;
    if (angle >= 0) {
      constant = 1;
    } else {
      constant = -1;
    }
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    _driveTrain.resetGyro();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    error =  angle - _driveTrain.getNavAngle(); 
    error = (error / angle);
    speed = error * 0.7;
    if (speed > 0.7) {
      speed = 0.7;
    }
    if (speed < 0.1) {
      speed = 0.1;
    }
    _driveTrain.tankDrive(constant*speed, -constant*speed);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {
    _driveTrain.tankDrive(0, 0);
    _driveTrain.resetEncoders();
  }

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if (Math.abs(_driveTrain.getNavAngle()) >= Math.abs(angle)) {
      return true;
    }
    return false;
  }
}
