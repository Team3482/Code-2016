package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutoDrawbridge extends CommandGroup {

    public AutoDrawbridge() {
        requires(Robot.chassis);
    	requires(Robot.arm);
    	
    	addSequential(new ArmPositionRest());
    	Timer.delay(.5);
    	addSequential(new ArmPositionDrawReach());
    	Timer.delay(.5);
    	//addSequential(new ArmPositionDrawPress());
    	//move back and move arm slowly
    	//end in home
    }
}
