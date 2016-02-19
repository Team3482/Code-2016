package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutoSallyPort extends CommandGroup {

    public AutoSallyPort() {
        requires(Robot.chassis);
    	requires(Robot.arm);
    	
    	addSequential(new ArmPositionRest());
    	Timer.delay(.5);
    	addSequential(new ArmPositionSally());
    	Timer.delay(.5);
    	//move back and turn
    	//end in home
    }
}
