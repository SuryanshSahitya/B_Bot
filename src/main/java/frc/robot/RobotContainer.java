// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import static edu.wpi.first.units.Units.*;

import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.ctre.phoenix6.swerve.SwerveRequest;

import edu.wpi.first.epilogue.Logged;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.units.measure.AngularVelocity;
import edu.wpi.first.units.measure.LinearVelocity;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.utils.TunableController;
import frc.robot.utils.TunableController.TunableControllerType;
import edu.wpi.first.wpilibj2.command.button.RobotModeTriggers;
import edu.wpi.first.wpilibj2.command.sysid.SysIdRoutine.Direction;
import org.littletonrobotics.junction.networktables.LoggedDashboardChooser;


import frc.robot.generated.TunerConstants;
import frc.robot.subsystems.CommandSwerveDrivetrain;
// import frc.robot.subsystems.camera;
import frc.robot.subsystems.climb;
import frc.robot.subsystems.intake;

public class RobotContainer {
 

// to do list:
// Setpoints
// tuning




    private static LinearVelocity MaxSpeed = TunerConstants.kSpeedAt12Volts; // kSpeedAt12Volts desired top speed
    private static final double MaxSpeed_double = MaxSpeed.in(MetersPerSecond);
    public static final AngularVelocity MaxAngularRate = RotationsPerSecond.of(3);
    public static final double MaxAngularRate_double = MaxAngularRate.in(RadiansPerSecond);
    //private double MaxAngularRate = RotationsPerSecond.of(0.75).in(RadiansPerSecond); // 3/4 of a rotation per second max angular velocity
    //private double MaxSpeed = TunerConstants.kSpeedAt12Volts.in(MetersPerSecond); // kSpeedAt12Volts desired top speed

    /* Setting up bindings for necessary control of the swerve drive platform */
    private final SwerveRequest.FieldCentric drive =
        new SwerveRequest.FieldCentric()
            .withDeadband(MaxSpeed.times(0.3))
            .withRotationalDeadband(MaxAngularRate.times(0.3)) // Add a 10% deadband
            .withDriveRequestType(DriveRequestType.OpenLoopVoltage); // Use open-loop control for drive motors



    private final SwerveRequest.SwerveDriveBrake brake = new SwerveRequest.SwerveDriveBrake();
    private final SwerveRequest.PointWheelsAt point = new SwerveRequest.PointWheelsAt();
    private final SwerveRequest.RobotCentric robotCentric_Translation = new SwerveRequest.RobotCentric();

    private final Telemetry logger = new Telemetry(MaxSpeed_double);

    private final TunableController driverJoy = new TunableController(1).withControllerType(TunableControllerType.QUADRATIC);
    private final CommandXboxController operatorJoy = new CommandXboxController(1);

    public final CommandSwerveDrivetrain drivetrain = TunerConstants.createDrivetrain();

    private final intake Intake = new intake();
    private final climb climb = new climb();
    // private final camera camera = new camera();
    
    // private LoggedDashboardChooser<Command> autoSelected =
    //     new LoggedDashboardChooser<>("Auto Selected", AutoBuilder.buildAutoChooser());

    private final double Outtake_position1 = 3.9404296875; // NEEDED TO BE CHANGED
    private final double Outtake_position2 = 4; // NEEDED TO BE CHANGED



    public RobotContainer() {
        configureBindings();

        NamedCommands.registerCommand("Outake Position1", 
        new SequentialCommandGroup(
            Intake.pivotCommand(Outtake_position1), // NEEDED TO BE CHANGED
            Intake.intakemotorSpeed(-1)));



        //     NamedCommands.registerCommand("Intake",
        //     new ConditionalCommand(
        //         // If robot is IDLE (not holding a piece): start intake
        //         new SequentialCommandGroup(
        //             new ParallelCommandGroup(
        //                 Intake.pivotCommand(13.999),  // lower intake
        //                 Intake.runOnce(() -> Intake.intakeSpeed(0.5)) // spin intake inwards
        //             ),
        //             // Wait until robot detects that piece is intaked (robot state changes to IDLE)
        //             Commands.waitUntil(() -> Constants.getRobotState() == Constants.RobotState.IDLE)
        //                 .withTimeout(3), // safety timeout (optional)
        //             // Once intaked, slow intake and retract
        //             new ParallelCommandGroup(
        //                 Intake.pivotCommand(0.2),
        //                 Intake.runOnce(() -> Intake.intakeSpeed(0.05))
        //             )
        //         ),
        //         // ELSE: if robot is not IDLE (already has a piece or shooting)
        //         new ParallelCommandGroup(
        //             Intake.pivotCommand(0.2),
        //             Intake.runOnce(() -> Intake.intakeSpeed(0.05))
        //         ),
        //         // Condition: should we intake? → intake if NOT currently holding a piece
        //         () -> Constants.getRobotState() != Constants.RobotState.IDLE
        //     )
        // );
        
        NamedCommands.registerCommand("Intake",
            new SequentialCommandGroup(
                // Step 1: Start intake and pivot down
                new ParallelCommandGroup(
                    Intake.pivotCommand(13.999),
                    Intake.intakemotorSpeed(0.5))
                ,

                // Step 2: Wait until the piece is fully intaked (state = IDLE)
                Commands.waitUntil(() -> Constants.getRobotState() == Constants.RobotState.IDLE),

                // Step 3: Pivot up and hold piece gently
                new ParallelCommandGroup(
                    Intake.pivotCommand(0.2),
                    Intake.intakemotorSpeed(0.05)
                )
            ));


        // NamedCommands.registerCommand("Intake", 
        // Commands.run( () -> 
        // {
        // new ConditionalCommand(
        //     new ParallelCommandGroup(
        //         Intake.pivotCommand(13.999),
        //         Intake.runOnce(() -> Intake.intakeSpeed(0.5))), 
        //     new ParallelCommandGroup(
        //         Intake.pivotCommand(0.2),
        //         Intake.runOnce(() -> Intake.intakeSpeed(0.05))), 
        //     () -> Constants.getRobotState() == Constants.RobotState.IDLE).schedule();
        // }
        // )
        // );

        //Another way of doing it
    //     NamedCommands.registerCommand("Intake", 
    //     Commands.run(() -> {
    //     if (Constants.getRobotState() == Constants.RobotState.IDLE) {
    //         Intake.pivotCommand(13.999).schedule();
    //         Intake.intakeSpeed(0.5);
    //     } else {
    //         Intake.pivotCommand(0.2).schedule();
    //         Intake.intakeSpeed(0.05);
    //    }}));






    }

    private void configureBindings() {
        // Note that X is defined as forward according to WPILib convention,
        // and Y is defined as to the left according to WPILib convention.
        drivetrain.setDefaultCommand(
            // Drivetrain will execute this command periodically
            drivetrain.applyRequest(() ->
                drive.withVelocityX(driverJoy.customLeft().getY() * MaxSpeed_double) // Drive forward with negative Y (forward)
                    .withVelocityY(driverJoy.customLeft().getX() * MaxSpeed_double) // Drive left with negative X (left)
                    .withRotationalRate(-driverJoy.customRight().getX() * MaxAngularRate_double)// Drive counterclockwise with negative X (left)
                    .withDeadband(0.3)
                    .withRotationalDeadband(0.3) 
                    )
        );

        // Idle while the robot is disabled. This ensures the configured
        // neutral mode is applied to the drive motors while disabled.
        final var idle = new SwerveRequest.Idle();
        RobotModeTriggers.disabled().whileTrue(
            drivetrain.applyRequest(() -> idle).ignoringDisable(true)
        );

        driverJoy.a().whileTrue(drivetrain.applyRequest(() -> brake));
        driverJoy.b().whileTrue(drivetrain.applyRequest(() ->
            point.withModuleDirection(new Rotation2d(-driverJoy.getLeftY(), -driverJoy.getLeftX()))
        ));

        
        // DRIVE CONTROLS


        // Run SysId routines when holding back/start and X/Y.
        // Note that each routine should be run exactly once in a single log.
        driverJoy.back().and(driverJoy.y()).whileTrue(drivetrain.sysIdDynamic(Direction.kForward));
        driverJoy.back().and(driverJoy.x()).whileTrue(drivetrain.sysIdDynamic(Direction.kReverse));
        driverJoy.start().and(driverJoy.y()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kForward));
        driverJoy.start().and(driverJoy.x()).whileTrue(drivetrain.sysIdQuasistatic(Direction.kReverse));

        // reset the field-centric heading on left bumper press
        driverJoy.x().onTrue(drivetrain.runOnce(() -> drivetrain.seedFieldCentric()));

        drivetrain.registerTelemetry(logger::telemeterize);


        // alignment swerve
        driverJoy.pov(0)
            .whileTrue(
                drivetrain.applyRequest(() -> robotCentric_Translation.withVelocityX(0.8).withVelocityY(0)));

        driverJoy.pov(180)
            .whileTrue(
                drivetrain.applyRequest(() -> robotCentric_Translation.withVelocityX(-0.8).withVelocityY(0)));
        
        driverJoy.pov(90)
            .whileTrue(
                drivetrain.applyRequest(() -> robotCentric_Translation.withVelocityX(0).withVelocityY(-0.8)));

        driverJoy.pov(270)
            .whileTrue(
                drivetrain.applyRequest(() -> robotCentric_Translation.withVelocityX(0).withVelocityY(0.8)));


        // climb down w setpoint
        driverJoy.leftBumper()
            .onTrue(
                new SequentialCommandGroup(
                Intake.pivotCommand(13.999),
                climb.climbPivotCommand(100), // needed to be changed
                climb.runOnce(() -> climb.climbIntake(0.2))));

        // climb up w setpoint
        driverJoy.rightBumper()
            .onTrue(climb.climbPivotCommand(10)); // needed to be changed


//sdfg

        // Run climb motor manueally
        climb.setDefaultCommand(
            Commands.run(() -> 
                climb.climbPivotSpeed(
                    driverJoy.getLeftTriggerAxis() * 0.7 + driverJoy.getRightTriggerAxis() * -0.7), 
                    climb));

        //OPERATOR CONTROLS

        // algea intake
        operatorJoy.x()
            .whileTrue(
                new ParallelCommandGroup(
                    Intake.pivotCommand(11.7),
                    Intake.intakemotorSpeed(0.67)
                )
            ).whileFalse(
                new ParallelCommandGroup(
                    Intake.pivotCommand(0.2),
                    Intake.intakemotorSpeed(0.1)
                )
            );

        // algea outtake
        operatorJoy.a()
            .whileTrue(
                new ParallelCommandGroup(
                    Intake.pivotCommand(9),
                    Intake.intakemotorSpeed(-1)
                )
            ).whileFalse(
                new ParallelCommandGroup(
                    Intake.pivotCommand(0.2),
                    Intake.intakemotorSpeed(0)
                )
            );

        // intake
        operatorJoy.leftTrigger(0.2)
            .whileTrue(
                new ParallelCommandGroup(
                Intake.pivotCommand(13.999),
                Intake.intakemotorSpeed(0.5)))
            .whileFalse(
                new ParallelCommandGroup(
                Intake.pivotCommand(0.2), // needed to changed
                Intake.intakemotorSpeed(0.05)));

        // outtake position 1
        operatorJoy.rightTrigger(.2)
                .whileTrue(
                    new ParallelCommandGroup(
                    Intake.pivotCommand(Outtake_position1), // NEEDED TO BE CHANGED
                    Intake.intakemotorSpeed(-1)))
                .whileFalse(
                    new ParallelCommandGroup(
                    Intake.pivotCommand(0.2),
                    Intake.intakemotorSpeed(0)));

        // outtake position 2
        operatorJoy.b()
                .whileTrue(
                    new ParallelCommandGroup(
                    Intake.pivotCommand(Outtake_position2), // needed to changed
                    Intake.intakemotorSpeed(-1)))
                .whileFalse(
                    new ParallelCommandGroup(
                    Intake.pivotCommand(0.2),
                    Intake.intakemotorSpeed(0)));

        // just pivot intake
        operatorJoy.y()
            .onTrue(Intake.pivotCommand(13.999));

    }


    public Command getAutonomousCommand() {
        return Commands.print("Nothing here");
        //return autoSelected.get();
    }
}
