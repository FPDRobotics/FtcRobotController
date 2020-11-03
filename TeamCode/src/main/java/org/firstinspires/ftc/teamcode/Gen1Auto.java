package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous (name = "Gen1Auto")
public class Gen1Auto extends LinearRobot {
    @Override

    public void runOpMode() {

        super.runOpMode();

        waitForStart();

            telemetry.addData("Analysis", pipeline.getAnalysis());
            telemetry.addData("Rings", pipeline.position);
            telemetry.update();
            sleep(50);

            lowerWobbleArm();

            sleep(1000);



        if (pipeline.position.equals(EasyOpenCV.UltimateGoalPipeline.ringPosition.FOUR)) {
            //lowerWobbleArm();
            ComputeMotorPowers(0,.35,0,500);
            ComputeMotorPowers(.35,0,0, 750);
            sleep(500);
            wobbleClaw.setPosition(1);
            sleep(500);
            wobbleArm.setTargetPosition(-425);
            sleep(1000);
            ComputeMotorPowers(0,-.35,0,500);
            ComputeMotorPowers(.7,0,0,9500);
            lowerWobbleArm();
            wobbleClaw.setPosition(0);
            wobbleArm.setTargetPosition(-425);
            ComputeMotorPowers(-.7,0,0,2500);
        } else if (pipeline.position.equals(EasyOpenCV.UltimateGoalPipeline.ringPosition.ONE)) {
            //lowerWobbleArm();
            ComputeMotorPowers(0,.35,0,500);
            ComputeMotorPowers(.35,0,0, 750);
            sleep(500);
            wobbleClaw.setPosition(1);
            sleep(500);
            wobbleArm.setTargetPosition(-425);
            sleep(1000);
            ComputeMotorPowers(0,-.35,0,500);
            ComputeMotorPowers(.7,0,0,8500);
            ComputeMotorPowers(0, .35, 0,1);
            lowerWobbleArm();
            wobbleClaw.setPosition(0);
            wobbleArm.setTargetPosition(-425);
            ComputeMotorPowers(-.7,0,0,2500);
        } else {
            //lowerWobbleArm();
            ComputeMotorPowers(0,.35,0,500);
            ComputeMotorPowers(.35,0,0, 750);
            sleep(500);
            wobbleClaw.setPosition(1);
            sleep(500);
            wobbleArm.setTargetPosition(-425);
            sleep(1000);
            ComputeMotorPowers(0,-.35,0,500);
            ComputeMotorPowers(.7, 0, 0, 7000);
            lowerWobbleArm();
            wobbleClaw.setPosition(0);
            ComputeMotorPowers(.7,0,0,8500);
        }
    }
}