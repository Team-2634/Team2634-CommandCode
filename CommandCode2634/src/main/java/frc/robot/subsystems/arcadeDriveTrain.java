package frc.robot.subsystems;

import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorController;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;

public class sub_arcadeDrive extends SubsystemBase {
  private static DifferentialDrive m_robotDrive;
  private MotorControllerGroup m_leftSide;
  private MotorControllerGroup m_rightSide;

  public DifferentialDrive getRobotDrive(double xSpeed, double zRotation) {
    return m_robotDrive.arcadeDrive(xSpeed, zRotation, false);
    return m_robotDrive;
  }

  public sub_arcadeDrive(MotorController leftFront, MotorController rightFront, MotorController leftBack, MotorController rightBack) {
    m_leftSide = new MotorControllerGroup(leftFront, leftBack);
    m_rightSide = new MotorControllerGroup(rightFront, rightBack);
    m_robotDrive = new DifferentialDrive(m_leftSide, m_rightSide);
  }

  @Override
  public void periodic() {
  }
}
