package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutonomousCommand extends CommandGroup {

    public AutonomousCommand() {
        requires(Robot.chassis);
        //addSequential(new Move(-0.6, 0.0, 0.1), 3.0);
//        Timer.delay(1.0);
        // only do this if we are waiting for another robot to pass - angles will be messed up
        if (SmartDashboard.getBoolean("Waiting")) {
        	addSequential(new Move(0.7, 0, 20 * SmartDashboard.getNumber("Position")), 10.0);
        	addSequential(new Rotate(90, false), 5.0);
	        if (SmartDashboard.getBoolean("Cheval de Fris")) {
	        	Robot.intake.setTargetPortcullis();
	        } else if(SmartDashboard.getBoolean("Low Bar")) {
	        	Robot.intake.setTargetLower();
	        } else if (SmartDashboard.getBoolean("Port Cullis")) {
	        	addSequential(new Move(.7, 0, 3), 5.0);
	        	Robot.intake.setTargetPortcullis();
	        	addSequential(new Move(.7, 0, 5), 5.0);
	        	Robot.intake.setTargetRest();
	        	addSequential(new Move(.5, 0, 12));
	        } else if(SmartDashboard.getBoolean("GUN IT")) {
	        	addSequential(new Move(-1, 0, 80), 5);
	        }
	        addSequential(new Move(0.7, 0, 40), 5.0);
	        addSequential(new Rotate(180, false), 5.0);
	        addSequential(new Move(0.7, 0, 40), 5.0);
	        addSequential(new Rotate(90, false), 5.0);
        } else {
        	if (SmartDashboard.getBoolean("Cheval de Fris")) {
	        	Robot.intake.setTargetPortcullis();
	        //} else if(SmartDashboard.getBoolean("Low Bar")) {
	        } else if(false) {
	        	addParallel(new Move(0.7, 0, 150), 5.0);
	        	addSequential(new LowerIntake());
	        } else if (SmartDashboard.getBoolean("Port Cullis")) {
	        	Robot.intake.setTargetPortcullis();
	        	addSequential(new Move(.7, 0, 5), 5.0);
	        	Robot.intake.setTargetRest();
	        	addSequential(new Move(.5, 0, 12));
	        } //else if(SmartDashboard.getBoolean("GUN IT")) {
	        else if(true) {
	        	System.out.println("GUNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNNN");
	        	addSequential(new Move(1, 0, -1000), 2);
	        }
	        //addSequential(new Move(0.7, 0, 150), 5.0);
	        //addSequential(new Rotate(90, false), 5.0);
	        //addSequential(new Move(0.7, 0, 40), 5.0);
	        //addSequential(new Rotate(0, false), 5.0);
        }
        /*addSequential(new Move(0.7, 0, 40), 5.0);
        addSequential(new AutoShoot());*/
        
    }

    /*// Called just before this Command runs the first time
    protected void initialize() {
    	System.out.println("Moving");
    	//Robot.chassis.setDesiredDistance(12);
    }

    // Called repeatedly when this Command is scheduled to run
    protected void execute() {
    	//Robot.chassis.moveDistance();
    	Robot.chassis.move(0.4, 0.0);
    	Timer.delay(5.0);
    	Robot.chassis.move(0.0, 0.5);
    	Timer.delay(0.5);
    	Robot.chassis.move(0.0, 0.0);
    }

    // Make this return true when this Command no longer needs to run execute()
    protected boolean isFinished() {
        //return Robot.chassis.shouldBeMoving();
    	return false;
    }

    // Called once after isFinished returns true
    protected void end() {
    	//Robot.chassis.disableRotation();
    	//Robot.chassis.stop();
    }

    // Called when another command which requires one or more of the same
    // subsystems is scheduled to run
    protected void interrupted() {
    	end();
    }*/
}
