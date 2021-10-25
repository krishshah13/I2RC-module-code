// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveTrain;

public class DriveForward extends CommandBase {
  private final DriveTrain _driveTrain;
  private double distance;

  /** Creates a new TimedAuto. */
  public DriveForward(DriveTrain dt, double newDistance) {
    _driveTrain = dt;
    distance = newDistance;
    // Use addRequirements() here to declare subsystem dependencies.
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
      _driveTrain.tankDrive(0.5, 0.5);
    
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
    if (_driveTrain.getDistance() >= distance) {
      return true;
    }
    return false;
  }
}
