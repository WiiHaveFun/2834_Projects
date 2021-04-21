// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class Pneumatics extends SubsystemBase {
  // Compressor
  private final Compressor m_c;
  // Intake Arms Solenoid
  private final Solenoid m_solenoid;
  /** Creates a new Pneumatics. */
  public Pneumatics(int PCMID, int solenoidChannel) {
    // Init the compressor
    m_c = new Compressor(PCMID);
    m_c.setClosedLoopControl(true);

    // Init the solenoid
    m_solenoid = new Solenoid(PCMID, solenoidChannel);
    // Set default output
    m_solenoid.set(false);

    // Adds a boolean to the dashboard for manual compressor control
    SmartDashboard.putBoolean("Compressor", false);
  }

  /**
   * Sets the solenoids output
   * 
   * @param output the desired output of the solenoid
   */
  public void setSolenoidOutput(boolean output) {
    m_solenoid.set(output);
  }

  /**
   * Gets the solenoids output
   * 
   * @return the output of the solenoid
   */
  public boolean getSolenoidOutput() {
    return m_solenoid.get();
  }

  @Override
  public void periodic() {
    // Change the state of the compressor if the SmartDashboard button is opposite from the current state.
    if(m_c.enabled() && !SmartDashboard.getBoolean("Compressor", true)) {
      m_c.stop();
    } else if(!m_c.enabled() && SmartDashboard.getBoolean("Compressor", true)) {
      m_c.start();
    }
  }
}
