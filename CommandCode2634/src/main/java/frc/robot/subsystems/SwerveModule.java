package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.sensors.WPI_CANCoder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.RobotController;
import frc.robot.Constants.DriveConstants;
import frc.robot.Constants.ModuleConstants;

public class SwerveModule {

    private final WPI_TalonFX driveMotor;
    private final WPI_TalonFX turningMotor;
    private final WPI_CANCoder absTurningEncoder;

    double talonDriveEncoderValue_posRad;
    double talonDriveEncoderVel_mps;
    double talonTurnEncoderValue_posRad;
    double talonTurnEncoderVel_mps;
    double absTurnEncoderValue;
    double absTurnEncoderVel;

    private final PIDController turningPidController;

    private final boolean absoluteEncoderReversed;
    private final double absoluteEncoderOffsetRad;

    public SwerveModule(int driveMotorId, int turningMotorId, boolean driveMotorReversed, boolean turningMotorReversed,
            int absoluteEncoderId, double absoluteEncoderOffsetRadi, boolean absoluteEncoderReversed) {

        this.absoluteEncoderOffsetRad = absoluteEncoderOffsetRadi;
        this.absoluteEncoderReversed = absoluteEncoderReversed;

        driveMotor = new WPI_TalonFX(driveMotorId);
        turningMotor = new WPI_TalonFX(turningMotorId);
        absTurningEncoder = new WPI_CANCoder(absoluteEncoderId);

        driveMotor.setInverted(driveMotorReversed);
        turningMotor.setInverted(turningMotorReversed);

        talonDriveEncoderValue_posRad = driveMotor.getSelectedSensorPosition()*ModuleConstants.talon_TurningEncoderTicksToRad;
        talonDriveEncoderVel_mps = driveMotor.getSelectedSensorVelocity()*ModuleConstants.talon_TurningEncoderTicksToMetresPerSec;
        talonTurnEncoderValue_posRad = turningMotor.getSelectedSensorPosition()*ModuleConstants.talon_TurningEncoderTicksToRad;
        talonTurnEncoderVel_mps = turningMotor.getSelectedSensorVelocity()*ModuleConstants.talon_TurningEncoderTicksToMetresPerSec;

        turningPidController = new PIDController(ModuleConstants.kPTurning, 0, 0);
        turningPidController.enableContinuousInput(-Math.PI, Math.PI);

        resetEncoders();
    }

    public double getDrivePosition() {
        return talonDriveEncoderValue_posRad;
    }

    public double getTurningPosition() {
        return talonTurnEncoderValue_posRad;
    }

    public double getDriveVelocity() {
        return talonDriveEncoderVel_mps;
    }

    public double getTurningVelocity() {
        return talonTurnEncoderVel_mps;
    }

    public double getAbsoluteEncoderRad() {
        return (absTurningEncoder.getAbsolutePosition() - absoluteEncoderOffsetRad) * (Math.PI/180);
        /*
        double angle = absTurningEncoder.getBusVoltage() / RobotController.getVoltage5V();
        angle *= 2.0 * Math.PI;
        angle -= absoluteEncoderOffsetRad;
        return angle * (absoluteEncoderReversed ? -1.0 : 1.0);
         */
    }

    public void resetEncoders() {
        driveMotor.setSelectedSensorPosition(0);
        turningMotor.setSelectedSensorPosition(getAbsoluteEncoderRad());
    }

    public SwerveModuleState getState() {
        return new SwerveModuleState(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public SwerveModulePosition getPos() {
        return new SwerveModulePosition(getDriveVelocity(), new Rotation2d(getTurningPosition()));
    }

    public void setDesiredState(SwerveModuleState state) {
        if (Math.abs(state.speedMetersPerSecond) < 0.001) {
            stop();
            return;
        }
        state = SwerveModuleState.optimize(state, getState().angle);
        driveMotor.set(state.speedMetersPerSecond / DriveConstants.kPhysicalMaxSpeedMetersPerSecond);
        turningMotor.set(turningPidController.calculate(getTurningPosition(), state.angle.getRadians()));
        //SmartDashboard.putString("Swerve[" + absoluteEncoder.getChannel() + "] state", state.toString());
    }

    public void stop() {
        driveMotor.set(0);
        turningMotor.set(0);
    }
}