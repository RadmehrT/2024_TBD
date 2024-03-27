package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.LEDs.LEDStates;
import frc.robot.Constants.Trap.SetpointsTrap;
import frc.robot.subsystems.LEDHandler;
import frc.robot.subsystems.Trap;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.Hopper;

public class TrapIndex extends Command {
    private Trap s_Trap;
    private Pivot s_Pivot;
    private FlyWheel s_FlyWheel;
    private Hopper s_Hopper;
    private LEDHandler s_LedHandler;
    
    public TrapIndex(Trap s_Trap, Pivot s_Pivot, FlyWheel s_FlyWheel, LEDHandler s_LedHandler) {
        this.s_Trap = s_Trap;
        this.s_Pivot = s_Pivot;
        this.s_FlyWheel = s_FlyWheel;
        this.s_LedHandler = s_LedHandler;

        addRequirements(s_Trap, s_Pivot, s_FlyWheel);
    }

    @Override
    public void initialize() {
        s_Trap.enable();
        s_Trap.requestGoal(SetpointsTrap.STOW);

        s_Pivot.enable();
        s_Pivot.handoffToTrap();
    }
    
    @Override 
    public void execute() {
        if (!s_Pivot.getController().atGoal()) {
            return;
        }

        s_Hopper.setMotorsSpeed(0, 0.3);
        s_FlyWheel.indexing();
        s_Trap.runIntake();


    }

    @Override
    public void end(boolean interrupted) {
        s_FlyWheel.idle();
        s_Hopper.stop();
        s_Pivot.returnHome();
        s_Trap.stopTrapIntake();

        s_LedHandler.request(LEDStates.AMP);
    }

    @Override
    public boolean isFinished() {
        return s_Trap.checkForTrapGamePiece();
    }
}
