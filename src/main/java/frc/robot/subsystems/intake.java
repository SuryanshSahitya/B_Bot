package frc.robot.subsystems;

import com.ctre.phoenix6.configs.MotionMagicConfigs;
import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.NeutralModeValue;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class intake extends SubsystemBase{
    public TalonFX pivotMotor = new TalonFX(9);
    public TalonFX intakeMotor = new TalonFX(10);
    TalonFXConfiguration talonFXConfiguration = new TalonFXConfiguration();
    private Slot0Configs slot0 = talonFXConfiguration.Slot0;
    MotionMagicConfigs motionMagicConfigs = talonFXConfiguration.MotionMagic;
    final MotionMagicVoltage m_request = new MotionMagicVoltage(0);



    public intake() {
      // motion magic stuff, comments are there for understanding
    slot0.kS = 0.15;
    slot0.kV = 0.10; // A velocity target of 1 rps results in 0.12 V output
    slot0.kA = 0.01; // An acceleration of 1 rps/s requires 0.01 V output
    slot0.kP = 0.01; // A position error of 2.5 rotations results in 12 V output
    slot0.kI = 0; // no output for integrated error
    slot0.kD = 0.0; // A velocity error of 1 rps results in 0.1 V output

    motionMagicConfigs.MotionMagicCruiseVelocity = 70; // Target cruise velocity of 80 rps
    motionMagicConfigs.MotionMagicAcceleration = 200; // Target acceleration of 160 rps/s (0.5 seconds)
    motionMagicConfigs.MotionMagicJerk = 400;// Target jerk of 1600 rps/s/s (0.1 seconds)


    pivotMotor.getConfigurator().apply(talonFXConfiguration); // aplies motion magic to the motor
    pivotMotor.setNeutralMode(NeutralModeValue.Brake); // setting the motor to brake when not used by driver
    intakeMotor.setNeutralMode(NeutralModeValue.Coast);

    }


    @Override
    public void periodic(){
        SmartDashboard.putNumber("Pivot motor position", pivotMotor.getPosition().getValueAsDouble());  
        SmartDashboard.putNumber("Intake motor current", intakeMotor.getStatorCurrent().getValueAsDouble());
        SmartDashboard.putNumber("Intake motor velocity", intakeMotor.getVelocity().getValueAsDouble());

        if (intakeMotor.getVelocity().getValueAsDouble() >= 100) {
          Constants.setRobotState(Constants.RobotState.SHOOTING);
        } else if (intakeMotor.getStatorCurrent().getValueAsDouble() <= 20 && intakeMotor.getVelocity().getValueAsDouble() <= -100) {
          Constants.setRobotState(Constants.RobotState.INTAKE);
        } else if (intakeMotor.getStatorCurrent().getValueAsDouble() > 20) {
          Constants.setRobotState(Constants.RobotState.DRIVE);
        }
    }

    public void intakeSpeed(double speed) {
        intakeMotor.set(speed);
    }


    public Command pivotCommand(double position) {
        return new Command() {
            @Override
            public void initialize() {
              // Initialization code, such as resetting encoders or PID controllers
              
              
              
              //PID Way of doing it
              //ClimbDownPID.setSetpoint(position);
            }
      
            @Override
            public void execute() {
            
            
            // PID way of doing it
              // double speed = ClimbDownPID.calculate(climbmotor.getPosition().getValueAsDouble());
              // climbmotor.set(speed);


            // Motion Magic way of doing it
            pivotMotor.setControl(m_request.withPosition(position).withFeedForward(0.15).withEnableFOC(true));
            //the feedforward is not required it just helps it being more personalized, like multiplying it by a constant

            }
      
            @Override
            public void end(boolean interrupted) {
              pivotMotor.setVoltage(0);


            }
      
            @Override
            public boolean isFinished() {
              return false;
            }
          };
    }
}
