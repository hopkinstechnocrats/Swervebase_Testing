// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.
//TODO: add position between L2 and bottom for elevator
//TODO: corrections offset for elevator
//TODO: end effector keep steady whiile driving
//TODO: add climber
//TODO: merge in auto branch
//TODO: use absolute encoder for end effector
//TODO: make end effector rights deeper
//TODO: actively NOT updating libraries

package frc.robot;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.reduxrobotics.canand.CanandEventLoop;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.Constants.elevatorConstants;
import frc.robot.Constants.endEffectorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import java.io.File;
import swervelib.SwerveInputStream;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a "declarative" paradigm, very
 * little robot logic should actually be handled in the {@link Robot} periodic methods (other than the scheduler calls).
 * Instead, the structure of the robot (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer
{
  // Replace with CommandPS4Controller or CommandJoystick if needed
  final CommandXboxController driverXbox = new CommandXboxController(0);
  final CommandXboxController operatorController = new CommandXboxController(1);
  // The robot's subsystems and commands are defined here...
  private final SwerveSubsystem       drivebase  = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), "swerve"));

  private SendableChooser<Command> m_chooser = new SendableChooser<>();

  private Boolean robot_score_left = true;


  /**
   * Converts driver input into a field-relative ChassisSpeeds that is controlled by angular velocity.
   */
  SwerveInputStream driveAngularVelocity = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                () -> driverXbox.getLeftY() *-1,
                                                                () -> driverXbox.getLeftX() *-1)
                                                            .withControllerRotationAxis(driverXbox::getRightX)
                                                            .deadband(OperatorConstants.DEADBAND)
                                                            .scaleTranslation(0.8)
                                                            .allianceRelativeControl(true);

  SwerveInputStream driveAngularVelocity_slow = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                            () -> driverXbox.getLeftY() * -1 * 0.5 ,
                                                            () -> driverXbox.getLeftX() * -1 * 0.5 )
                                                        .withControllerRotationAxis(driverXbox::getRightX)
                                                        .deadband(OperatorConstants.DEADBAND)
                                                        .scaleTranslation(0.8)
                                                        .allianceRelativeControl(true);
                                                      
  /**
   * Clone's the angular velocity input stream and converts it to a fieldRelative input stream.
   */
  SwerveInputStream driveDirectAngle = driveAngularVelocity.copy().withControllerHeadingAxis(() -> headingX(),
                                                                                             () -> headingY())
                                                           .headingWhile(true);

  SwerveInputStream driveDirectAngle_slow = driveAngularVelocity_slow.copy().withControllerHeadingAxis(() -> headingX() ,
                                                                                                  () -> headingY() )
                                                           .headingWhile(true);
  /**
   * Clone's the angular velocity input stream and converts it to a robotRelative input stream.
   */
  SwerveInputStream driveRobotOriented = driveAngularVelocity.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);
  SwerveInputStream driveRobotOriented_slow = driveAngularVelocity_slow.copy().robotRelative(true)
                                                             .allianceRelativeControl(false);

  SwerveInputStream driveAngularVelocityKeyboard = SwerveInputStream.of(drivebase.getSwerveDrive(),
                                                                        () -> -driverXbox.getLeftY(),
                                                                        () -> -driverXbox.getLeftX())
                                                                    .withControllerRotationAxis(() -> driverXbox.getRawAxis(
                                                                        2))
                                                                    .deadband(OperatorConstants.DEADBAND)
                                                                    .scaleTranslation(0.8)
                                                                    .allianceRelativeControl(true);
  // Derive the heading axis with math!
  SwerveInputStream driveDirectAngleKeyboard = driveAngularVelocityKeyboard.copy()
                        .withControllerHeadingAxis(() ->Math.sin(
                            driverXbox.getRawAxis(2) * Math.PI) *
                            (Math.PI * 2),
                        () -> Math.cos(
                            driverXbox.getRawAxis(2) * Math.PI) *
                            (Math.PI * 2)).headingWhile(true);

  /**
   * The container for the robot. Contains subsystems, OI devices, and commands.
   */
  public RobotContainer()
  {
    // Configure the trigger bindings
    CanandEventLoop.getInstance();
    configureBindings();
    DriverStation.silenceJoystickConnectionWarning(true);
    NamedCommands.registerCommand("test", Commands.print("I EXIST"));
    boolean isCompetition = false;
    m_chooser = AutoBuilder.buildAutoChooser();

    SmartDashboard.putData("Auto Chooser", m_chooser);
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary predicate, or via the
   * named factories in {@link edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for
   * {@link CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller PS4}
   * controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight joysticks}.
   */
  private void configureBindings()
  {
    Command driveFieldOrientedDirectAngle      = drivebase.driveFieldOriented(driveDirectAngle);
    Command driveFieldOrientedDirectAngle_slow      = drivebase.driveFieldOriented(driveDirectAngle_slow);
    Command driveFieldOrientedAnglularVelocity = drivebase.driveFieldOriented(driveAngularVelocity);
    Command driveFieldOrientedAnglularVelocity_slow = drivebase.driveFieldOriented(driveAngularVelocity_slow);
    Command driveRobotOrientedAngularVelocity  = drivebase.driveFieldOriented(driveRobotOriented);
    Command driveSetpointGen = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngle);
    Command driveFieldOrientedDirectAngleKeyboard      = drivebase.driveFieldOriented(driveDirectAngleKeyboard);
    Command driveFieldOrientedAnglularVelocityKeyboard = drivebase.driveFieldOriented(driveAngularVelocityKeyboard);
    Command driveSetpointGenKeyboard = drivebase.driveWithSetpointGeneratorFieldRelative(
        driveDirectAngleKeyboard);

    if (RobotBase.isSimulation())
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngleKeyboard);
    } else
    {
      drivebase.setDefaultCommand(driveFieldOrientedDirectAngle);
    }

    if (Robot.isSimulation())
    {
      driveDirectAngleKeyboard.driveToPose(() -> new Pose2d(new Translation2d(9, 3),
                                                            Rotation2d.fromDegrees(90)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(5,
                                                                                     3)),
                                           new ProfiledPIDController(5,
                                                                     0,
                                                                     0,
                                                                     new Constraints(
                                                                         Math.toRadians(
                                                                             360),
                                                                         Math.toRadians(
                                                                             90))));
      driverXbox.start().onTrue(Commands.runOnce(() -> drivebase.resetOdometry(new Pose2d(3, 3, new Rotation2d()))));
      driverXbox.button(1).whileTrue(drivebase.sysIdDriveMotorCommand());
      driverXbox.button(2).whileTrue(Commands.runEnd(() -> driveDirectAngleKeyboard.driveToPoseEnabled(true),
                                                     () -> driveDirectAngleKeyboard.driveToPoseEnabled(false)));

    }
    if (DriverStation.isTest())
    {
      drivebase.setDefaultCommand(driveFieldOrientedAnglularVelocity); // Overrides drive command above!

      driverXbox.x().whileTrue(Commands.runOnce(drivebase::lock, drivebase).repeatedly());
      driverXbox.y().whileTrue(drivebase.driveToDistanceCommand(1.0, 0.2));
      driverXbox.start().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.back().whileTrue(drivebase.centerModulesCommand());
      driverXbox.leftBumper().onTrue(Commands.none());
      driverXbox.rightBumper().onTrue(Commands.none());
    } else
    {
      driverXbox.a().onTrue((Commands.runOnce(drivebase::zeroGyro)));
      driverXbox.leftTrigger().whileTrue((driveFieldOrientedDirectAngle_slow));
      driverXbox.start().whileTrue(new RunCommand(() -> increaseOffset()));
      driverXbox.back().whileTrue(new RunCommand(() -> decreaseOffset()));
      driverXbox.povUp().onTrue(new RunCommand(() -> resetOffset()));
      driverXbox.leftBumper().whileTrue((driveFieldOrientedAnglularVelocity));
      }
}
  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand()
  {
    // Run selected auto
    return m_chooser.getSelected();

  }

  public void setMotorBrake(boolean brake)
  {
    drivebase.setMotorBrake(brake);
  }

  public double offset = 0.0;

  public void increaseOffset(){
    offset = offset + Math.PI/36;
  }

  public void decreaseOffset(){
    offset = offset - Math.PI/36;
  }

  public void resetOffset(){
    offset = 0.0;
  }


  public Double headingX(){
    final double deadband = 0.50;
    final double num_sections = 6;
    final double rad_per_section = (2.0*Math.PI/num_sections);
    final double section =((((Math.atan2(-driverXbox.getRightY(), driverXbox.getRightX())))/rad_per_section)+0.5); // in from -2.5 to +3.5
    final int section_rounded = Math.round((float) section);
    final double angle = section_rounded * rad_per_section;
    // System.out.println(angle);
    // System.out.println(section_rounded);
    if( Math.hypot(driverXbox.getRightX(),-driverXbox.getRightY()) <= deadband){
      // System.out.println("inside deadband");
      return 0.0;
    } else {
    return Math.cos(angle+ ((4*Math.PI)/3)) ; // TODO add 120 degrees to angle
    }
  }




  public Double headingY(){
    final double deadband = 0.50;
    final double num_sections = 6;
    final double rad_per_section = (2.0*Math.PI/num_sections);
    final double section =((((Math.atan2(-driverXbox.getRightY(), driverXbox.getRightX())))/rad_per_section)+0.5); // in from -2.5 to +3.5
    final int section_rounded = Math.round((float) section);
    final double angle = section_rounded * rad_per_section;
    if( Math.hypot(driverXbox.getRightX(),-driverXbox.getRightY()) <= deadband){
      return 0.0;
    } else {
    return -Math.sin(angle+ ((4*Math.PI)/3));
} 
}
}
