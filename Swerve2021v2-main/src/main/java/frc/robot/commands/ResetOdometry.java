// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.DriveSubsystem;

public class ResetOdometry extends CommandBase {
  DriveSubsystem m_driveSubsystem;
  Trajectory m_trajectory;
  boolean resetGyro;

  /** 
   * Creates a new ResetOdometry. 
   * 
   * @param driveSubsystem The drive subsytem of the robot
   * @param trajectory The trajectory whose start point needs to be referenced
   * @param resetGyro Boolean for if the gyro needs to be reset
   * */
  public ResetOdometry(DriveSubsystem driveSubsystem, Trajectory trajectory, boolean resetGyro) {
    // Use addRequirements() here to declare subsystem dependencies.
    m_driveSubsystem = driveSubsystem;
    m_trajectory = trajectory;
    this.resetGyro = resetGyro;

    addRequirements(m_driveSubsystem);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    if(resetGyro) {
      m_driveSubsystem.resetGyro();
    }
    m_driveSubsystem.resetOdometry(m_trajectory.getInitialPose());
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return true;
  }
}
