// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;

import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
import frc.robot.subsystems.climb;
import frc.robot.subsystems.intake;

public class RobotContainer {
    private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed
    private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive = new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed * 0.3).withRotationalDeadband(MaxAngularRate * 0.3) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors
    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();

    private final Telemetry logger = new Telemetry(MaxSpeed);

    private final CommandXboxController driverJoy = new CommandXboxController(0);
    private final CommandXboxController operatorJoy = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final intake Intake = new intake();
    private final climb climb = new climb();



    public RobotContainer() {
        configureBindings();
    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverJoy.getLeftY() * MaxSpeed) // Drive forward with negative Y (forward)
                    .withVelocityY(driverJoy.getLeftX() * MaxSpeed) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoy.getRightX() * MaxAngularRate) // Drive counterclockwise with negative X (left)
                    )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        //driverJoy.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoy.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoy.getLeftY(), -driverJoy.getLeftX()))
        ));

        

        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoy.back().and(driverJoy.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoy.back().and(driverJoy.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoy.start().and(driverJoy.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoy.start().and(driverJoy.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoy.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);



        //operator controls
        // intake
        driverJoy.leftBumper()
            .whileTrue(
                new ParallelCommandGroup(
                Intake.pivotCommand(13.5),
                Intake.runOnce(() -> Intake.intakeSpeed(0.5))))
            .whileFalse(
                new ParallelCommandGroup(
                Intake.pivotCommand(0.5), // needed to changed
                Intake.runOnce(() -> Intake.intakeSpeed(0.05))));
// outtake position 1
            driverJoy.rightBumper()
                .whileTrue(
                    new ParallelCommandGroup(
                    Intake.pivotCommand(4.77), // NEEDED TO BE CHANGED
                    Intake.runOnce(() -> Intake.intakeSpeed(-1))))
                .whileFalse(
                    new ParallelCommandGroup(
                    Intake.pivotCommand(0.5),
                    Intake.runOnce(() -> Intake.intakeSpeed(0))));

    // outtake position 2
    driverJoy.a()
    .whileTrue(
        new ParallelCommandGroup(
        Intake.pivotCommand(0.5), // needed to changed
        Intake.runOnce(() -> Intake.intakeSpeed(-1))))
    .whileFalse(
        new ParallelCommandGroup(
        Intake.pivotCommand(0.5),
        Intake.runOnce(() -> Intake.intakeSpeed(0))));

    }

    public Command getAutonomousCommand() {
        return Commands.print("No autonomous command configured");
    }
}
