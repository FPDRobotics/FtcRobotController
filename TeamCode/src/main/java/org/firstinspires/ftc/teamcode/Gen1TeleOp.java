package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//Teleop shows up under telop dropdown on phone with that name in green
@TeleOp (name = "Gen1TeleOp")
public class Gen1TeleOp extends LinearRobot {

    @Override
    public void runOpMode() {

        super.runOpMode();

        waitForStart();

        while (opModeIsActive()) {

            //drive

            drive();

            //arm movement
            keyFinder();

            //Wobble button positions
            if (gamepad1.right_bumper) {
                wobbleClaw.setPosition(1);
            } else if (gamepad1.left_bumper) {
                wobbleClaw.setPosition(0);
            }

            if (gamepad1.dpad_left){
                ringClaw.setPosition(1);
            } else if (gamepad1.dpad_right) {
                ringClaw.setPosition(0);
            }

            if(gamepad1.dpad_down){
                lowerWobbleArm();
            } else if (gamepad1.dpad_up){
                raiseWobbleArm();
            }

            //Telemetry
            telemetry.update();
        }
    }
}