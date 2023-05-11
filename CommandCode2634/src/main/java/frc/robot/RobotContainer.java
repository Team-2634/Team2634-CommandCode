package frc.robot;

import java.util.List;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SwerveControllerCommand;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Constants.AutoConstants;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.OIConstants;
import frc.robot.commands.cmd_SwerveDriveJoySticks;
import frc.robot.commands.cmd_arcadeDrive;
import frc.robot.subsystems.arcadeDriveTrain;
import frc.robot.subsystems.sub_SwerveDrive;
import frc.robot.subsystems.sub_arcadeDrive;

public class RobotContainer {
  final Constants cont = new Constants();
  //private final arcadeDriveTrain arcadeDriveTrain_Neo = new arcadeDriveTrain(cont.leftFrontMax, cont.rightFrontMax, cont.leftBackMax, cont.rightBackMax);
  //private final arcadeDriveTrain arcadeDriveTrain_Talon = new arcadeDriveTrain(cont.leftFrontFX, cont.rightFrontFX, cont.leftBackFX, cont.rightBackFX);
  private final sub_SwerveDrive swerveSubsystem = new sub_SwerveDrive();
  private final Joystick driverJoytick = new Joystick(OIConstants.kDriverControllerPort);
  private final sub_arcadeDrive m_robotArcadeDrive = new sub_arcadeDrive();
  private final cmd_arcadeDrive arcadeDrivingCom = new cmd_arcadeDrive(m_robotArcadeDrive, cont.xBox0);

  public RobotContainer() {
    swerveSubsystem.setDefaultCommand(new cmd_SwerveDriveJoySticks(
      swerveSubsystem,
      () -> -driverJoytick.getRawAxis(OIConstants.kDriverYAxis),
      () -> driverJoytick.getRawAxis(OIConstants.kDriverXAxis),
      () -> driverJoytick.getRawAxis(OIConstants.kDriverRotAxis),
      () -> !driverJoytick.getRawButton(OIConstants.kDriverFieldOrientedButtonIdx)));

    configureBindings();
  }

  private void configureBindings() {
    m_robotArcadeDrive.setDefaultCommand(arcadeDrivingCom);
    new JoystickButton(driverJoytick, 2).whenPressed(() -> swerveSubsystem.zeroHeading());
  }

  public Command getAutonomousCommand() {
    // 1. Create trajectory settings
    TrajectoryConfig trajectoryConfig = new TrajectoryConfig(
            AutoConstants.kMaxSpeedMetersPerSecond,
            AutoConstants.kMaxAccelerationMetersPerSecondSquared)
                    .setKinematics(DriveConstants.kDriveKinematics);

    // 2. Generate trajectory
    Trajectory trajectory = TrajectoryGenerator.generateTrajectory(
            new Pose2d(0, 0, new Rotation2d(0)),
            List.of(
                    new Translation2d(1, 0),
                    new Translation2d(1, -1)),
            new Pose2d(2, -1, Rotation2d.fromDegrees(180)),
            trajectoryConfig);

    // 3. Define PID controllers for tracking trajectory
    PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    ProfiledPIDController thetaController = new ProfiledPIDController(
            AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    thetaController.enableContinuousInput(-Math.PI, Math.PI);

    // 4. Construct command to follow trajectory
    SwerveControllerCommand swerveControllerCommand = new SwerveControllerCommand(
            trajectory,
            swerveSubsystem::getPose,
            DriveConstants.kDriveKinematics,
            xController,
            yController,
            thetaController,
            swerveSubsystem::setModuleStates,
            swerveSubsystem);

    // 5. Add some init and wrap-up, and return everything
    return new SequentialCommandGroup(
      new InstantCommand(() -> swerveSubsystem.resetOdometry(trajectory.getInitialPose())),
      swerveControllerCommand,
      new InstantCommand(() -> swerveSubsystem.stopModules()));
  }
}