
package frc.robot;

import com.ctre.phoenix.sensors.CANCoder;
import com.fasterxml.jackson.annotation.JacksonInject.Value;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import frc.robot.Constants.DriveConstants;

public class Robot extends TimedRobot {

  private Command m_autonomousCommand;
  //private RobotContainer m_robotContainer;
  private SwerveContainer m_swervContainer;

  @Override
  public void robotInit() {
  //m_robotContainer = new RobotContainer();
  m_swervContainer = new SwerveContainer();
  }
  CANCoder absTurningEncoderTL = new CANCoder(3);
  CANCoder absTurningEncoderBL = new CANCoder(2);
  CANCoder absTurningEncoderBR = new CANCoder(1);
  CANCoder absTurningEncoderTR = new CANCoder(0);
  
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
    SmartDashboard.putNumber("absTL", (absTurningEncoderTL.getAbsolutePosition() * (Math.PI/180)));
    double value =  (absTurningEncoderTL.getAbsolutePosition() * (Math.PI/180)) - DriveConstants.kFrontLeftDriveAbsoluteEncoderOffsetRad;
    SmartDashboard.putNumber("absTLDC", value);


    /*
    SmartDashboard.putNumber("absTR", (absTurningEncoderTR.getAbsolutePosition() * (Math.PI/180)));
    SmartDashboard.putNumber("absTRDC", (absTurningEncoderTR.getAbsolutePosition() - DriveConstants.kFrontRightDriveAbsoluteEncoderOffsetRad) * (Math.PI/180));
    SmartDashboard.putNumber("absBL", (absTurningEncoderBL.getAbsolutePosition() * (Math.PI/180)));
    SmartDashboard.putNumber("absBLDC", (absTurningEncoderBL.getAbsolutePosition() - DriveConstants.kBackLeftDriveAbsoluteEncoderOffsetRad) * (Math.PI/180));
    SmartDashboard.putNumber("absBR", (absTurningEncoderBR.getAbsolutePosition() * (Math.PI/180)));
    SmartDashboard.putNumber("absBRDC", (absTurningEncoderBR.getAbsolutePosition() - DriveConstants.kBackRightDriveAbsoluteEncoderOffsetRad) * (Math.PI/180));
   */
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void autonomousInit() {
    //m_autonomousCommand = m_robotContainer.getAutonomousCommand();
    m_autonomousCommand = m_swervContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
