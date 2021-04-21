// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc.robot.Trajectories;
import frc.robot.Constants.BrushConstants;
import frc.robot.Constants.IntakeConstants;
import frc.robot.subsystems.Brush;
import frc.robot.subsystems.DriveSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Pneumatics;
import frc.robot.subsystems.Socket;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class SearchAuto extends SequentialCommandGroup {
  Trajectories trajectories;

  /** Creates a new SearchAuto. */
  public SearchAuto(DriveSubsystem m_driveSubsystem, Pneumatics m_pneumatics, Intake m_intake, Brush m_brush, Socket m_socket) {
    trajectories = new Trajectories();

    // Add your commands in the addCommands() call, e.g.
    // addCommands(new FooCommand(), new BarCommand());
    addCommands(new SetPickUpTrajectory(m_socket, trajectories),
                new ResetOdometry(m_driveSubsystem, trajectories.slalomTrajectory, true),
                new ControlIntake(m_intake, IntakeConstants.kIntakePower),
                new ControlBrush(m_brush, BrushConstants.kNominalPower),
                new ControlPneumatics(m_pneumatics, true),
                new SchedulePath(m_driveSubsystem, trajectories),
                new StopDrivetrain(m_driveSubsystem),
                new ControlIntake(m_intake, IntakeConstants.kNeutralPower));
  }
}
