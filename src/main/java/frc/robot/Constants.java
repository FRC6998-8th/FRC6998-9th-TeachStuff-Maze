// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkBase;
import com.revrobotics.spark.SparkBase.PersistMode;
import com.revrobotics.spark.SparkBase.ResetMode;
import com.revrobotics.spark.config.ClosedLoopConfig;
import com.revrobotics.spark.config.SparkMaxConfig;
import com.revrobotics.spark.config.ClosedLoopConfig.FeedbackSensor;
import com.revrobotics.spark.config.SparkBaseConfig.IdleMode;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static class Motors {
    public static final double Pitch_kS = 0.01/12;
    public static final double Pitch_kG = 0.02/12;
    public static final double Pitch_kV = 0.1/12;
    public static final double Pitch_kA = 0;

    public static final int ROLL_DEVICE_ID = 1;
    public static final int PITCH_DEVICE_ID = 2;

    public static PersistMode kPersist = SparkBase.PersistMode.kPersistParameters;
    public static ResetMode kReset = SparkBase.ResetMode.kResetSafeParameters;

    public static SparkMaxConfig configRoll = new SparkMaxConfig();
    public static SparkMaxConfig configPitch = new SparkMaxConfig();

    public static final double GEAR_RATIO = 1.0 / (9.0 * 85.0 / 35.0);

    public static final boolean isRollInversted = false;
    public static final boolean isPitchInversted = true;

    public static final double ROLL_kP = 3;
    public static final double PITCH_kP = 3;
    public static final double ROLL_FF = 0.0;
    public static final double PITCH_FF = 0.0;

    public static final double OneSideLimit = 0.035;

    public final static double ForwardLimit = OneSideLimit*2;
    public final static double ReverseLimit = -0.0001;

    // public final static double RollLimit = 0.035; //rotation

    static {
      configRoll.idleMode(IdleMode.kBrake).smartCurrentLimit(60).inverted(isRollInversted);      
      configPitch.idleMode(IdleMode.kBrake).smartCurrentLimit(60).inverted(isPitchInversted);

      configRoll.encoder.velocityConversionFactor(GEAR_RATIO/60);
      configRoll.encoder.positionConversionFactor(GEAR_RATIO);

      configRoll.softLimit
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(ForwardLimit)
        .reverseSoftLimit(ReverseLimit);
      
      configRoll.closedLoop
        .apply(new ClosedLoopConfig().pidf(ROLL_kP, 0, 0, ROLL_FF))
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);

      configPitch.encoder.velocityConversionFactor(GEAR_RATIO/60);
      configPitch.encoder.positionConversionFactor(GEAR_RATIO);

      configPitch.softLimit
        .forwardSoftLimitEnabled(true)
        .reverseSoftLimitEnabled(true)
        .forwardSoftLimit(ForwardLimit)
        .reverseSoftLimit(ReverseLimit);

      configPitch.closedLoop
        .apply(new ClosedLoopConfig().pidf(PITCH_kP, 0, 0, PITCH_FF))
        .feedbackSensor(FeedbackSensor.kPrimaryEncoder)
        .outputRange(-1, 1);
    }
  }

  public static class OperatorConstants {
    public static final int kDriverControllerPort = 0;
  }
}
