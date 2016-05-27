package org.usfirst.frc3482.robot.commands;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.CommandGroup;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import org.usfirst.frc3482.robot.Robot;

/**
 *
 */
public class AutonomousLowBar extends CommandGroup {
	
    public AutonomousLowBar() {
        requires(Robot.chassis);
        requires(Robot.arm);
        requires(Robot.intake);
        requires(Robot.camera);

        System.out.println("LOWWWWWW");
    	addSequential(new LowerTargetIntake());
    	addSequential(new Move(0.7, 0, 10000), 3.0);
    	addSequential(new RestTargetIntake());
    }
    
}
