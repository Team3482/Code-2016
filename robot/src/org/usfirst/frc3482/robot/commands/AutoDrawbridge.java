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
    	addSequential(new Wait(20)); 
    	addSequential(new ArmPositionDrawReach());
    	addSequential(new Wait(20));
    	addSequential(new ArmPositionDrawPress());
    	//addSequential(new Move(-0.7, 0.0, 30));
    	//move back and move arm slowly
    	//end in home
    }

}
