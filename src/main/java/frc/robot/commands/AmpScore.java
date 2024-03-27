package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Constants.Trap.SetpointsTrap;
import frc.robot.subsystems.LEDHandler;
import frc.robot.subsystems.Trap;

public class AmpScore extends Command {
    private Trap s_Trap;
    private LEDHandler s_LedHandler;

    public AmpScore(Trap s_Trap, LEDHandler s_LedHandler) {
        this.s_Trap = s_Trap;
        this.s_LedHandler = s_LedHandler;

        addRequirements(s_Trap);
    }

    @Override
    public void initialize() {
        s_Trap.enable();
        s_Trap.requestGoal(SetpointsTrap.AMP);
    }

    @Override
    public void execute() {
        if (s_Trap.getController().atGoal()) {
            s_Trap.runIntake();
        }
    }

    @Override
    public void end(boolean interrupted) {
        //TODO: LEDs
        s_Trap.stopTrapIntake();
        s_Trap.requestGoal(SetpointsTrap.STOW);
    }
}
