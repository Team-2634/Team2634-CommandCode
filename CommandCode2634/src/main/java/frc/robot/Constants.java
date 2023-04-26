
package frc.robot;

public final class Constants {
    public static final int DriverControllerPort = 0;

    public final CANSparkMax leftFrontMax = new CANSparkMax(4, MotorType.kBrushless);
    public final CANSparkMax rightFrontMax = new CANSparkMax(10, MotorType.kBrushless);
    public final CANSparkMax leftBackMax = new CANSparkMax(17, MotorType.kBrushless);
    public final CANSparkMax rightBackMax = new CANSparkMax(18, MotorType.kBrushless);
}