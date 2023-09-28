// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import java.util.function.DoubleSupplier;

import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.motorcontrol.VictorSP;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Drivetrain extends SubsystemBase {
  private VictorSP frontLeftMotor;
  private VictorSP frontRightMotor;
  private VictorSP backLeftMotor;
  private VictorSP backRightMotor;

  private MotorControllerGroup left;
  private MotorControllerGroup right;
  private DifferentialDrive dd;

  private SlewRateLimiter limiter = new SlewRateLimiter(1.0);

  /** Creates a new Drivetrain. */
  public Drivetrain() {
    frontLeftMotor = new VictorSP(Constants.FRONT_LEFT_MOTOR);
    frontRightMotor = new VictorSP(Constants.FRONT_RIGHT_MOTOR);
    backLeftMotor = new VictorSP(Constants.BACK_LEFT_MOTOR);
    backRightMotor = new VictorSP(Constants.BACK_RIGHT_MOTOR);
    left = new MotorControllerGroup(backLeftMotor, frontLeftMotor);
    right = new MotorControllerGroup(backRightMotor, frontRightMotor);
    dd = new DifferentialDrive(left, right);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public Command drive(DoubleSupplier x, DoubleSupplier z) {
    return runEnd(() -> { dd.arcadeDrive(limiter.calculate(z.getAsDouble()) * 0.5, x.getAsDouble() * 0.5); }, () -> { dd.arcadeDrive(0, 0); });
  }
}
