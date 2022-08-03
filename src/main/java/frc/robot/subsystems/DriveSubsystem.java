// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import org.apache.commons.math3.analysis.polynomials.PolynomialSplineFunction;

import com.ctre.phoenix.motorcontrol.StatorCurrentLimitConfiguration;
import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.utils.CoaxialSwerveModule;
import frc.robot.utils.TractionControlController;
import frc.robot.utils.TurnPIDController;

public class DriveSubsystem extends SubsystemBase implements AutoCloseable {
  public static class Hardware {
    CoaxialSwerveModule lFrontModule;
    CoaxialSwerveModule rFrontModule;
    CoaxialSwerveModule lRearModule;
    CoaxialSwerveModule rRearModule;
    AHRS navx;

    public Hardware(CoaxialSwerveModule lFrontModule, CoaxialSwerveModule rFrontModule,
                    CoaxialSwerveModule lRearModule, CoaxialSwerveModule rRearModule,
                    AHRS navx) {
      this.lFrontModule = lFrontModule;
      this.rFrontModule = rFrontModule;
      this.lRearModule = lRearModule;
      this.rRearModule = rRearModule;
      this.navx = navx;
    }
  }

  private String SUBSYSTEM_NAME = "Drive Subsystem";

  private TurnPIDController m_turnPIDController;
  private TractionControlController m_tractionControlController;
  private SwerveDriveKinematics m_kinematics;
  private SwerveDriveOdometry m_odometry;

  private CoaxialSwerveModule m_lFrontModule;
  private CoaxialSwerveModule m_rFrontModule;
  private CoaxialSwerveModule m_lRearModule;
  private CoaxialSwerveModule m_rRearModule;

  private AHRS m_navx;

  private final double TOLERANCE = 0.125;
  private final double VISION_AIM_DAMPENER = 0.9;
 
  private double m_metersPerTick = 0.0;
  private double m_deadband = 0.0;

  /**
   * Create an instance of DriveSubsystem
   * <p>
   * NOTE: ONLY ONE INSTANCE SHOULD EXIST AT ANY TIME!
   * <p>
   * @param drivetrainHardware Hardware devices required by drivetrain
   * @param kP Proportional gain
   * @param kD Derivative gain
   * @param turnScalar Scalar for turn input (degrees)
   * @param deadband Deadband for controller input [+0.001, +0.1]
   * @param lookAhead Turn PID lookahead, in number of loops
   * @param metersPerTick Meters traveled per encoder tick (meters)
   * @param maxLinearSpeed Maximum linear speed of the robot (m/s)
   * @param tractionControlCurve Spline function characterising traction of the robot
   * @param throttleInputCurve Spline function characterising throttle input
   * @param turnInputCurve Spline function characterising turn input
   * @param currentLimitConfiguration Drive current limit
   */
  public DriveSubsystem(Hardware drivetrainHardware, double kP, double kD, double turnScalar, double deadband, double lookAhead,
                        double metersPerTick, double maxLinearSpeed, 
                        PolynomialSplineFunction tractionControlCurve, PolynomialSplineFunction throttleInputCurve, PolynomialSplineFunction turnInputCurve,
                        StatorCurrentLimitConfiguration currentLimitConfiguration) {
    m_turnPIDController = new TurnPIDController(kP, kD, turnScalar, deadband, lookAhead, turnInputCurve);
    m_tractionControlController = new TractionControlController(deadband, maxLinearSpeed, tractionControlCurve, throttleInputCurve);

    this.m_lFrontModule = drivetrainHardware.lFrontModule;
    this.m_rFrontModule = drivetrainHardware.rFrontModule;
    this.m_lRearModule = drivetrainHardware.lRearModule;
    this.m_rRearModule = drivetrainHardware.rRearModule;
    
    CoaxialSwerveModule.setTractionControlController(m_tractionControlController);

    this.m_navx = drivetrainHardware.navx;

    this.m_deadband = deadband;
    this.m_metersPerTick = metersPerTick;

    m_turnPIDController.setTolerance(TOLERANCE);

    m_navx.calibrate();
    m_navx.reset();

    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_lFrontModule.moduleLocation,
                                                                   m_rFrontModule.moduleLocation,
                                                                   m_lRearModule.moduleLocation,
                                                                   m_rRearModule.moduleLocation);

    // Initialise odometry
    m_odometry = new SwerveDriveOdometry(m_kinematics, new Rotation2d());
  }

  public static Hardware initializeHardware(double wheelbase, double trackWidth) {
    CoaxialSwerveModule lFrontModule = new CoaxialSwerveModule(CoaxialSwerveModule.initializeHardware(1, 2),
                                                               new Translation2d(+wheelbase / 2.0, +trackWidth / 2.0));
    CoaxialSwerveModule rFrontModule = new CoaxialSwerveModule(CoaxialSwerveModule.initializeHardware(3, 4),
                                                               new Translation2d(+wheelbase / 2.0, -trackWidth / 2.0));
    CoaxialSwerveModule lRearModule = new CoaxialSwerveModule(CoaxialSwerveModule.initializeHardware(5, 6),
                                                               new Translation2d(-wheelbase / 2.0, +trackWidth / 2.0));
    CoaxialSwerveModule rRearModule = new CoaxialSwerveModule(CoaxialSwerveModule.initializeHardware(7, 8),
                                                               new Translation2d(-wheelbase / 2.0, -trackWidth / 2.0));
    Hardware drivetrainHardware = new Hardware(lFrontModule, rFrontModule, lRearModule, rRearModule, new AHRS(SPI.Port.kMXP));

    return drivetrainHardware;
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void teleop(double velocityX, double velocityY, double rotateRate) {
    // Convert speeds to module states
    SwerveModuleState[] moduleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(velocityX, velocityY, Math.toRadians(rotateRate)));

    // Set modules to calculated states
    m_lFrontModule.set(moduleStates);
    m_rFrontModule.set(moduleStates);
    m_lRearModule.set(moduleStates);
    m_rRearModule.set(moduleStates);
  }

  @Override
  public void close() {

  }
}
