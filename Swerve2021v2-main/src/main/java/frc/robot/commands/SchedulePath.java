// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveSubsystem;

public class SchedulePath extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  Trajectories trajectories;
  SwerveControllerCommand command;
  /** Creates a new SchedulePath. */
  public SchedulePath(DriveSubsystem driveSubsystem, Trajectories trajectories) {
    m_driveSubsystem = driveSubsystem;
    this.trajectories = trajectories;
    // Use addRequirements() here to declare subsystem dependencies.
    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    if(trajectories.variableTrajectory != null) {
      command = m_driveSubsystem.getSwerveControllerCommand(trajectories.variableTrajectory, trajectories.thetaController);
      command.schedule();
    }
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {}

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    if(command.isFinished()) {
      return true;
    } else {
      return false;
    }
  }
}
