package org.firstinspires.ftc.teamcode.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.subsystems.Drivetrain;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Vision;

public class RobotHardware {
    /* Declare OpMode members. */
    private LinearOpMode myOpMode = null;   
    public final Drivetrain drivetrain;

    public final Intake intake;
    public final Vision vision;

    /* Constructor */
    public RobotHardware(LinearOpMode opmode) {
        myOpMode = opmode;
        drivetrain = new Drivetrain(myOpMode.hardwareMap);
        vision = new Vision(myOpMode.hardwareMap);
        intake = new Intake(myOpMode.hardwareMap);
    }

    /**
     * Initialize all the robot's hardware.
     * This method must be called ONCE when the OpMode is initialized.
     */
    public void init() {
        myOpMode.telemetry.addData(">", "Hardware Initialized");
        myOpMode.telemetry.update();
    }

    /**
     * Stop all motors
     */
    public void stop() {
        try {
            drivetrain.stop();
            intake.stop();
            vision.close();
        } catch (Exception e) {
            myOpMode.telemetry.addData("Error during stop", e.getMessage());
            myOpMode.telemetry.update();
        }
    }
} 