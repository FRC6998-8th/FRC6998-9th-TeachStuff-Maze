package frc.robot.subsystems;

import com.studica.frc.AHRS;
import com.studica.frc.AHRS.NavXComType;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Gyroscope extends SubsystemBase {
  private final AHRS gyro;

  public Gyroscope() {
    gyro = new AHRS(NavXComType.kMXP_SPI);    
  }

  public double GetDriveRoll() {
    double tmp = Range(-Constants.Motors.OneSideLimit, Constants.Motors.OneSideLimit,
    gyro.getRoll()/360) + Constants.Motors.OneSideLimit;

    return Range(0.001, Constants.Motors.ForwardLimit-0.001, tmp);
  }

  public double GetDrivePitch() {
    double tmp = Range(-Constants.Motors.OneSideLimit, Constants.Motors.OneSideLimit,
    gyro.getPitch()/360) + Constants.Motors.OneSideLimit;

    return Range(0.0001, Constants.Motors.ForwardLimit-0.001, tmp);
  }

  private double Range(double min, double max, double n) {
    return Math.min(Math.max(n, min), max);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    SmartDashboard.putBoolean("isconnected", gyro.isConnected());
    SmartDashboard.putBoolean("isCalibrating", gyro.isCalibrating());
    SmartDashboard.putNumber("port number", gyro.getPort());
    SmartDashboard.putNumber("yaw", gyro.getYaw());
    SmartDashboard.putNumber("update count", gyro.getUpdateCount());
    SmartDashboard.putNumber("update rate", gyro.getActualUpdateRate());
    SmartDashboard.putNumber("DriveRoll", gyro.getRoll());
    SmartDashboard.putNumber("DrivePitch", gyro.getPitch());
  }

  @Override
  public void simulationPeriodic() {
    // This mellllthod will be called once per scheduler run during simulation

  }
}
