package frc.robot.subsystems;

import java.util.function.Supplier;

import com.ctre.phoenix6.controls.DutyCycleOut;
import com.ctre.phoenix6.controls.VoltageOut;
import com.ctre.phoenix6.hardware.TalonFX;

import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj2.command.ProfiledPIDSubsystem;
import frc.robot.Constants;
import frc.robot.Constants.Intake.Setpoints;
import frc.robot.Constants.Trap.SetpointsTrap;

public class Trap extends ProfiledPIDSubsystem {

    private final TalonFX m_TrapArmMotor = new TalonFX(Constants.Trap.TRAP_MOTOR_ID_0);
    private final TalonFX m_TrapIntakeMotor = new TalonFX(Constants.Trap.TRAP_MOTOR_ID_1);

    private boolean isHomed = false;
    private boolean isRunning = false;
    private boolean ampMode = false;

    private VoltageOut m_TrapArmRequest = new VoltageOut(0.0);
    private DutyCycleOut m_TrapIntakeRequest = new DutyCycleOut(0.0);

    private Supplier<Boolean> m_CollisionAvoidanceSupplier;    

    public Trap(Supplier<Boolean> m_ShouldMoveTrap) {
        super(new ProfiledPIDController(
            Constants.Trap.TRAP_P, 
            0,
            0,
            new TrapezoidProfile.Constraints(0, 0))            
        );

        getController().setTolerance(0.03);
        this.m_CollisionAvoidanceSupplier = m_ShouldMoveTrap;        
    }

    @Override
    protected double getMeasurement() {
        return getTrapArmPosition();
    }

    @Override
    protected void useOutput(double output, TrapezoidProfile.State setpoint) {
        m_TrapArmMotor.setControl(m_TrapArmRequest.withOutput(output));
    }

    public double getTrapArmPosition() {
        return ((m_TrapArmMotor.getPosition()
            .getValueAsDouble()) / 104)*(2*Math.PI); //TODO replace with actual gear ratio
    }



    public void requestGoal(SetpointsTrap DesiredPosition) {

        if (!isHomed) {
            DriverStation.reportWarning(
                "WARNING: TRAP GOAL WAS REQUESTED WHILE TRAP IS NOT HOMED.",
                false
            );
            return;
        }

        switch (DesiredPosition) {
            case TRAP:
                isRunning = true;
                ampMode = false;
                break;
            case STOW:
                isRunning = false;
                ampMode = false;
                break;
            case AMP:
                ampMode = true;
                isRunning = false;
                break;
            default:
                isRunning = false;
                ampMode = false;
        }
    }

    public void runIntake() {
        m_TrapIntakeMotor.setControl(m_TrapIntakeRequest.withOutput(-0.7));
    }
    public void runIntakeReverse() {
        m_TrapIntakeMotor.setControl(m_TrapIntakeRequest.withOutput(0.7));
    }
    /* TODO: Tune AMP mode intake speed. */
    public void runTrapForAmp() {
        m_TrapIntakeMotor.setControl(m_TrapIntakeRequest.withOutput(0.6));
    }

    public void stopTrapIntake() {
        m_TrapIntakeMotor.stopMotor();
    }

    public void stopTrapArm() {
        m_TrapArmMotor.stopMotor();
    }

    /* Used for physical button on robot */
    public void setTrapAsHomed() {
        m_TrapIntakeMotor.setPosition(0.0);
        isHomed = true;
    }

    public void moveTrapTowardsGoal() {
        m_TrapArmMotor.setControl(m_TrapArmRequest);
    }

    public boolean isHomed() {
        return isHomed;
    }

    public void moveTrapTowardsHome() {
        setGoal(0.01);
    }

    @Override
    public void periodic() {
        super.periodic();

        if (!isHomed) {
            return;
        }

        if (m_CollisionAvoidanceSupplier.get()) {
            m_enabled = true;
            setGoal(0.0);
        } else if (isRunning) {
            setGoal(Constants.Trap.TrapSetpoints.get(SetpointsTrap.TRAP));
        } else if (ampMode) {
            setGoal(0); //TODO: Tune intake AMP mode setpoint
        } else {
            setGoal(Constants.Trap.TrapSetpoints.get(SetpointsTrap.STOW));
        }
    }
}
