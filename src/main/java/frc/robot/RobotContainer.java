// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

// import java.util.List;

// import com.pathplanner.lib.PathConstraints;
// import com.pathplanner.lib.PathPlanner;
// import com.pathplanner.lib.PathPlannerTrajectory;

import edu.wpi.first.wpilibj.Joystick;
import frc.robot.subsystems.Drivetrain;
import edu.wpi.first.wpilibj2.command.Command;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and button mappings) should be declared here.
 */
public class RobotContainer {
  // Subsystems
  final Drivetrain m_drivetrain = new Drivetrain();

  // Operator Input: two separate joysticks for tank drive (left and right)
  final Joystick leftStick = new Joystick(Constants.LEFT_JOYSTICK_PORT);
  final Joystick rightStick = new Joystick(Constants.RIGHT_JOYSTICK_PORT);

  // private final List<PathPlannerTrajectory> autoPg = PathPlanner.loadPathGroup("Autonomous", new PathConstraints(4, 3));

  public RobotContainer() {
    // Configure the button bindings
    configureButtonBindings();
  }

  /**
   * Use this method to define your button->command mappings. Buttons can be created by
   * instantiating a {@link GenericHID} or one of its subclasses ({@link
   * edu.wpi.first.wpilibj.Joystick} or {@link XboxController}), and then passing it to a {@link
   * edu.wpi.first.wpilibj2.command.button.JoystickButton}.
   */
  private void configureButtonBindings() {
  // Bind default command to left and right Y sticks for tank drive
  // Default command: left joystick Y controls left wheels, right joystick Y controls right wheels
  m_drivetrain.setDefaultCommand(m_drivetrain.drive(() -> leftStick.getY(), () -> rightStick.getY()));
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An TankDrive will run in autonomous
    return m_drivetrain.getDefaultCommand();
  }
}
