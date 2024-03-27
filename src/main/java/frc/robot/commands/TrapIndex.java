package frc.robot.commands;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.subsystems.Trap;
import frc.robot.subsystems.shooter.Pivot;
import frc.robot.subsystems.shooter.FlyWheel;
import frc.robot.subsystems.shooter.

public class TrapIndex extends Command {
    private Trap s_Trap;
    private Pivot s_Pivot;
    private FlyWheel s_FlyWheel;
    private Hopper s_Hopper;
    
    public TrapIndex(Trap s_Trap, Pivot s_Pivot, FlyWheel s_FlyWheel) {
        this.s_Trap = s_Trap;
        this.s_Pivot = s_Pivot;
        this.s_FlyWheel = s_FlyWheel;

        addRequirements(s_Trap, s_Pivot, s_FlyWheel);
    }

    @Override
    public void initialize() {
        s_Trap.enable();
        s_Pivot.enable();
        s_FlyWheel.run();
    }
    
    @Override
    public void end(boolean interrupted) {
        s_FlyWheel.stop();
    }
}
