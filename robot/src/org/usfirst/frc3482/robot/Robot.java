package org.usfirst.frc3482.robot;

import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.CANTalon;
import edu.wpi.first.wpilibj.Joystick.AxisType;
import edu.wpi.first.wpilibj.Joystick.RumbleType;
import edu.wpi.first.wpilibj.command.Command;
import edu.wpi.first.wpilibj.command.Scheduler;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.networktables.NetworkTable;

import java.io.IOException;

import org.usfirst.frc3482.robot.commands.*;
import org.usfirst.frc3482.robot.subsystems.*;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the IterativeRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the manifest file in the resource
 * directory.
 */
public class Robot extends IterativeRobot {

    Command autonomousCommand;
    Command teleopCommand;
    
    public static OI oi;
    public static Chassis chassis;
    public static Shooter shooter;
    public static Arm arm;
    public static Intake intake;
    public static Climber climber;
    public static Camera camera;
    
    private static final NetworkTable grip = NetworkTable.getTable("grip");
    public static double targetCenterX;
    public static double targetCenterY;
    
    
    /**
     * This function is run when the robot is first started up and should be
     * used for any initialization code.
     */
    public void robotInit() {
    	RobotMap.init();
        chassis = new Chassis();
        shooter = new Shooter();
        arm = new Arm();
        intake = new Intake();
        climber = new Climber();
        camera = new Camera();

        // OI must be constructed after subsystems. If the OI creates Commands
        //(which it very likely will), subsystems are not guaranteed to be
        // constructed yet. Thus, their requires() statements may grab null
        // pointers. Bad news. Don't move it.
        oi = new OI();
        chassis.invertMotors();
        
        try {
            new ProcessBuilder("/home/lvuser/grip").inheritIO().start();
        } catch (IOException e) {
            e.printStackTrace();
        }
        
        //instantiate the command used for the autonomous and teleop period
        autonomousCommand = new AutonomousCommand();
        teleopCommand = new Drive();
    }

    /**
     * This function is called when the disabled button is hit.
     * You can use it to reset subsystems before shutting down.
     */
    public void disabledInit(){

    }

    public void disabledPeriodic() {
        Scheduler.getInstance().run();
    	//System.out.println(Robot.chassis.rangeFinder.getAverageVoltage());
        //System.out.println(Robot.chassis.imu.getYaw());
    }

    public void autonomousInit() {
        // schedule the autonomous command (example)
        if (autonomousCommand != null) autonomousCommand.start();
        Robot.chassis.resetGyro();
        Robot.arm.setLowerRest();
        Robot.arm.setUpperHome();
    }

    /**
     * This function is called periodically during autonomous
     */
    public void autonomousPeriodic() {
        Scheduler.getInstance().run();
        
        Robot.intake.maintainPosition();
		Robot.arm.maintainLowerJointPosition();
        Robot.arm.maintainUpperJointPosition();
        
        int index = -1;
        double maxArea = -1;
        double[] areas = grip.getNumberArray("targets/area", new double[0]);
        double[] centerXs = grip.getNumberArray("targets/area", new double[0]);
        double[] centerYs = grip.getNumberArray("targets/area", new double[0]);
        
        for (int i=0;i<areas.length;i++) {
            System.out.println("Got contour with area=" + areas[i]);
            if(areas[i] > maxArea) {
            	index = i;
            	maxArea = areas[i];
            }
        }
        if(index != -1) {
        	targetCenterX = centerXs[index];
        	targetCenterY = centerYs[index];
        }
        System.out.println("Auto Loop");
        
    	//Robot.chassis.printRotateInfo();
		//move(0.0, rotateToAngleRate);
		//move(0.0,0.0);
	}
    

    public void teleopInit() {
        // This makes sure that the autonomous stops running when
        // teleop starts running. If you want the autonomous to
        // continue until interrupted by another command, remove
        // this line or comment it out.
        if (autonomousCommand != null) autonomousCommand.cancel();
        Robot.chassis.resetGyro();
        Robot.arm.setLowerRest();
        Robot.arm.setUpperHome();
    }

    /**
     * This function is called once each time the robot enters operator control.
     */
    public void operatorControl() {
        while (isOperatorControl() && isEnabled()) {
            chassis.imuData();
        }
     }
    
    /**
     * This function is called periodically during operator control
     */  
    public void teleopPeriodic() {
        Scheduler.getInstance().run();
        Robot.chassis.driveWithXboxController(Robot.oi.getxboxController());
        
        Robot.intake.maintainPosition();
        Robot.arm.maintainLowerJointPosition();
        Robot.arm.maintainUpperJointPosition();
        //System.out.println(Robot.chassis.imu.getYaw());
    }

    /**
     * This function is called periodically during test mode
     */
    public void testPeriodic() {
        LiveWindow.run();
        //Robot.arm.checkLower();
        //Robot.arm.checkUpper();
    }
}
