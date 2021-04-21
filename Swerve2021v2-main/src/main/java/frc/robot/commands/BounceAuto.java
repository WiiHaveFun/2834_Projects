// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Trajectories;
import frc.robot.subsystems.DriveSubsystem;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class BounceAuto extends SequentialCommandGroup {
  Trajectories trajectories;
  
  /** 
   * Creates a new BounceAuto. 
   * 
   * @param m_driveSubsystem The drive subsystem of the robot
   * */
  public BounceAuto(DriveSubsystem m_driveSubsystem) {
    trajectories = new Trajectories();
    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new ResetOdometry(m_driveSubsystem, trajectories.barrelTrajectory, true),
                m_driveSubsystem.getSwerveControllerCommand(trajectories.bounceTrajectory1, trajectories.thetaController),
                new StopDrivetrain(m_driveSubsystem),
                new ResetOdometry(m_driveSubsystem, trajectories.barrelTrajectory, false),
                m_driveSubsystem.getSwerveControllerCommand(trajectories.bounceTrajectory2, trajectories.thetaController),
                new StopDrivetrain(m_driveSubsystem),
                new ResetOdometry(m_driveSubsystem, trajectories.barrelTrajectory, false),
                m_driveSubsystem.getSwerveControllerCommand(trajectories.bounceTrajectory3, trajectories.thetaController),
                new StopDrivetrain(m_driveSubsystem),
                new ResetOdometry(m_driveSubsystem, trajectories.barrelTrajectory, false),
                m_driveSubsystem.getSwerveControllerCommand(trajectories.bounceTrajectory4, trajectories.thetaController),
                new StopDrivetrain(m_driveSubsystem));
  }
}
