package swervelib.imu;

import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;


public class IMUVelocity {
    private final SwerveIMU gyro;
    private final LinearFilter velocityFilter;
    private final Notifier  notifier;

    private boolean firstCycle = true;
    private double timestamp = 0.0;
    private Rotation2d position = new Rotation2d();
    private double velocity = 0.0;

    public IMUVelocity(SwerveIMU gyro)
    {
        this(gyro, 1.0/60.0, 5);
    }

    public IMUVelocity(SwerveIMU gyro, double periodSeconds, int averagingTaps)
    {
        this.gyro = gyro;
        velocityFilter = LinearFilter.movingAverage(averagingTaps);
        notifier = new Notifier(this::update);
        notifier.startPeriodic(periodSeconds);
        timestamp = RobotController.getFPGATime();
    }

    private void update() 
    {
        double newTimestamp = RobotController.getFPGATime();
        Rotation2d newPosition = Rotation2d.fromRadians(gyro.getRotation3d().getZ());

        synchronized (this) {
                if (!firstCycle) {
                    velocity = velocityFilter.calculate(
                    (newPosition.minus(position).getRadians()) / (newTimestamp - timestamp));
                }
                firstCycle = false;
                timestamp = newTimestamp;
                position = newPosition;
            }
    }

    public synchronized double getVelocity() {
        return velocity;
    }
}
