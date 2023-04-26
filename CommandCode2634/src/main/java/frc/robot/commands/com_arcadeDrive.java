package frc.robot.commands;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.subsystems.sub_arcadeDrive;

public class com_arcadeDrive extends CommandBase {
  private final XboxController m_Xstick;
  private final sub_arcadeDrive m_robotDrive;
  
  public com_arcadeDrive(sub_arcadeDrive robotDrive, XboxController Xstick) {
    m_Xstick = Xstick;
    m_robotDrive = robotDrive;
    addRequirements(m_robotDrive);
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {}

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    m_robotDrive.arcadeDrive(m_Xstick.getRawAxis(4) *0.8, m_Xstick.getRawAxis(1) * 0.8);
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
