package org.usfirst.frc3482.robot;

import org.usfirst.frc3482.robot.commands.*;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.buttons.*;
import org.usfirst.frc3482.robot.subsystems.*;


/**
 * This class is the glue that binds the controls on the physical operator
 * interface to the commands and command groups that allow control of the robot.
 */
public class OI {
    public JoystickButton spinAndShootButton;
    public JoystickButton armPositionBridgeButton;
    public JoystickButton lowerIntakeButton;
    public JoystickButton invertDirectionButton;
    public JoystickButton spinShooterButton;
    public JoystickButton armPositionPortButton;
    public JoystickButton shootButton;
    public JoystickButton forwardFeedButton; 
    public JoystickButton reverseFeedButton;
    public JoystickButton maintainIntakeButton;
    public JoystickButton topJointButton;
    public JoystickButton bridgeButton;
    public JoystickButton moveUpperJointButton;
    public JoystickButton moveLowerJointButton;
    public JoystickButton moveIntakeButton;
    public JoystickButton spinIntakeWheelsButton;
    public JoystickButton runWheelsOnGroundButton;
    public JoystickButton sallyPortButton;
    public JoystickButton portcullisButton;
    public JoystickButton armRestButton;
    public JoystickButton armHomeButton;
    public JoystickButton climberButton;
    public JoystickButton extendClimberButton;
    
    public Joystick xboxController;
    public Joystick joystick;
    public Joystick arcadeButtons;

    public OI() {
        xboxController = new Joystick(0);
        joystick = new Joystick(1);
        
        spinShooterButton = new JoystickButton(joystick, 2);
        spinShooterButton.whileHeld(new SpinShooter());
        
//        moveUpperJointButton = new JoystickButton(joystick, 10);
//        moveUpperJointButton.whileHeld(new MoveLowerJoint());
//        moveLowerJointButton = new JoystickButton(joystick, 9);
//        moveLowerJointButton.whileHeld(new MoveUpperJoint());
        moveIntakeButton = new JoystickButton(joystick, 6);
        moveIntakeButton.whileHeld(new MoveIntake());
        spinIntakeWheelsButton = new JoystickButton(joystick, 4);
        spinIntakeWheelsButton.whileHeld(new SpinIntakeWheels());
        runWheelsOnGroundButton = new JoystickButton(joystick, 7);
        runWheelsOnGroundButton.whileHeld(new RunWheelsOnGround());
        sallyPortButton = new JoystickButton(joystick, 11);
        sallyPortButton.whenPressed(new ArmPositionSally());
        //portcullisButton = new JoystickButton(joystick, 12);
        //portcullisButton.whileHeld(new ArmPositionPortCullis());
        armRestButton = new JoystickButton(joystick, 8);
        armRestButton.whenPressed(new ArmPositionRest());
        
        //climberButton = new JoystickButton(joystick, 7);
        //climberButton.whileHeld(new ClimbUp());
        //extendClimberButton = new JoystickButton(joystick, 8);
        //extendClimberButton.whileHeld(new ExtendClimber());
        
        
        invertDirectionButton = new JoystickButton(xboxController, 5);
        invertDirectionButton.whenPressed(new InvertDirection());
        forwardFeedButton = new JoystickButton(joystick, 1);
        forwardFeedButton.whileHeld(new FeedShooter());
        reverseFeedButton = new JoystickButton(joystick, 3);
        reverseFeedButton.whileHeld(new FeedShooter(false));
        lowerIntakeButton = new JoystickButton(joystick, 5);
        lowerIntakeButton.toggleWhenPressed(new LowerIntake());
//        bridgeButton = new JoystickButton(xboxController, 8);
//        bridgeButton.toggleWhenPressed(new ArmPositionBridge());
//        armPositionBridgeButton = new JoystickButton(xboxController, 2);
//        armPositionBridgeButton.whenPressed(new ArmPositionBridge());
        //spinAndShootButton = new JoystickButton(xboxController, 1);
        //spinAndShootButton.whenPressed(new SpinAndShoot());
        armHomeButton = new JoystickButton(joystick, 9);
        armHomeButton.whenPressed(new ArmPositionHome());
        //topJointButton.whileHeld(new );
        
        // SmartDashboard Buttons
        //SmartDashboard.putData("ArmPositionPort", new AutoSallyPort());
        //SmartDashboard.putData("ArmPositionBridge", new ArmPositionBridge());
        SmartDashboard.putData("InvertDirection", new InvertDirection());
        SmartDashboard.putData("LowerIntake", new LowerIntake());
        SmartDashboard.putData("SpinAndShoot", new SpinAndShoot());
        SmartDashboard.putData("Shoot", new Shoot());
        SmartDashboard.putData("SpinShooter", new SpinShooter());
        //SmartDashboard.putData("Move", new Move());
        SmartDashboard.putData("Autonomous Command", new AutonomousCommand());
        SmartDashboard.putData("Drive", new Drive());
    }

    public Joystick getxboxController() {
        return xboxController;
    }
    public Joystick getJoystick() {
    	return joystick;
    }
}

