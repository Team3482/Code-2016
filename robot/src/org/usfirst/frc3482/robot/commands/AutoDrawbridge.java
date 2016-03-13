package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;

import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutoDrawbridge extends Command {
	
	int loops;
	boolean finished;

    public AutoDrawbridge() {
        requires(Robot.chassis);
    	requires(Robot.arm);
    	requires(Robot.intake);
    	/*System.out.println("starting auto draw");
    	addSequential(new ArmPositionRest());
    	addSequential(new Wait(20)); 
    	System.out.println("rest/wait done");
    	addSequential(new ArmPositionDrawReach());
    	addSequential(new Wait(20));
    	System.out.println("reach/wait done");
    	addSequential(new ArmPositionDrawPress());
    	addSequential(new Wait(20));
    	System.out.println("press/wait done");
    	addSequential(new ArmPositionDrawPressMore());
    	System.out.println("press more done");
    	addSequential(new Wait(50));
    	addSequential(new ArmPositionHome());*/
    	//addSequential(new Move(-0.7, 0.0, 30));
    	//move back and move arm slowly
    	//end in home
    }
 // Called just before this Command runs the first time
    protected void initialize() {
    	loops = 0;
    	finished = false;
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	System.out.println("Execute"+loops);
    	if(loops == 0) {
    		System.out.println("loops --------------------------------- 0");
    		Robot.arm.setLowerRest();
    		Robot.arm.setUpperRest();
    	} else if(loops == 20) {
    		System.out.println("loops --------------------------------- 20");
    		Robot.arm.setLowerDrawReach();
        	Robot.arm.setUpperDrawReach();
    	} else if(loops == 20+20) {
    		System.out.println("loops --------------------------------- 40");
        	Robot.arm.setLowerDrawPress();
        	Robot.arm.setUpperDrawPress();
    	} else if(loops == 20+40) {
    		System.out.println("loops --------------------------------- 60");
        	Robot.arm.setLowerDrawPressMore();
    	} else if(loops == 50+60) {
    		System.out.println("loops --------------------------------- 110");
        	Robot.arm.setLowerRest();
        	Robot.arm.setUpperHome();
        	finished = true;
    	}
    	Robot.intake.maintainPosition();
    	Robot.arm.maintainLowerJointPosition();
    	Robot.arm.maintainUpperJointPosition();
    	loops++;
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        return finished;
    }

    // Called once after isFinished returns true
    protected void end() {
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }
}

