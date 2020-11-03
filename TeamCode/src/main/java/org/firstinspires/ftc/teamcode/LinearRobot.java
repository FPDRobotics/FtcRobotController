 package org.firstinspires.ftc.teamcode;

 import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
 import com.qualcomm.robotcore.hardware.CRServo;
 import com.qualcomm.robotcore.hardware.DcMotor;
 import com.qualcomm.robotcore.hardware.DcMotorSimple;
 import com.qualcomm.robotcore.hardware.Servo;

 import org.opencv.core.Core;
 import org.opencv.core.Mat;
 import org.opencv.core.Point;
 import org.opencv.core.Rect;
 import org.opencv.core.Scalar;
 import org.opencv.imgproc.Imgproc;
 import org.openftc.easyopencv.OpenCvCamera;
 import org.openftc.easyopencv.OpenCvCameraFactory;
 import org.openftc.easyopencv.OpenCvCameraRotation;
 import org.openftc.easyopencv.OpenCvInternalCamera;
 import org.openftc.easyopencv.OpenCvPipeline;

 public class LinearRobot extends LinearOpMode {

     // Declare the two back motors
     public OpenCvInternalCamera phoneCam;
     public EasyOpenCV.UltimateGoalPipeline pipeline;
     public DcMotor backLeftMotor, backRightMotor, wobbleArm, mainArm, frontLeftMotor, frontRightMotor;
     public Servo wobbleClaw, ringClaw;
     public CRServo mArmElbow;
     final private int raisedwobbleArm = -600;
     final private int loweredwobbleArm = 0;

     @Override

     public void runOpMode() {

         // Linking the motor variables with the actual hardware
         backLeftMotor = hardwareMap.dcMotor.get(RobotNames.BACK_LEFT_MOTOR);
         backRightMotor = hardwareMap.dcMotor.get(RobotNames.BACK_RIGHT_MOTOR);
         frontLeftMotor = hardwareMap.dcMotor.get(RobotNames.FRONT_LEFT_MOTOR);
         frontRightMotor = hardwareMap.dcMotor.get(RobotNames.FRONT_RIGHT_MOTOR);

         //camera
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
         pipeline = new EasyOpenCV.UltimateGoalPipeline();
         phoneCam.setPipeline(pipeline);

         phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

         phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
             @Override
             public void onOpened() {
                 phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
             }
         });


         wobbleArm = hardwareMap.dcMotor.get(RobotNames.WOBBLE_ARM);
         mainArm = hardwareMap.dcMotor.get(RobotNames.MAIN_ARM);

         //Linking the servo variables to the hardware
         wobbleClaw = hardwareMap.servo.get(RobotNames.WOBBLE_CLAW);
         ringClaw = hardwareMap.servo.get(RobotNames.RING_CLAW);
         mArmElbow = hardwareMap.crservo.get(RobotNames.M_ARM_ELBOW);

         //Run mode

         mainArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
         wobbleArm.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

         wobbleArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         mainArm.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         wobbleArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

         mainArm.setTargetPosition(0);
         wobbleArm.setTargetPosition(0);

         mainArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);

         // Reversing the RIGHT motors
         backRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);
         frontRightMotor.setDirection(DcMotorSimple.Direction.REVERSE);

         // Setting the motors to use encoders
         backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         frontRightMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
         backLeftMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

         backRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         backLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         frontRightMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
         frontLeftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
     }

     public void keyFinder() {

         int key = 0;

         if (gamepad1.y) {
             key = 4;
         } else if (gamepad1.x) {
             key = 1;
         } else if (gamepad1.a) {
             key = 2;
         } else if (gamepad1.b) {
             key = 3;
         }
         armMovement(key);
     }

     //arm movement function
     public void armMovement(int key) {
         if (key == 1) {
             mainArm.setTargetPosition(-328);

         } else if (key == 2) {
             mainArm.setTargetPosition(-552);

         } else if (key == 3) {
             mainArm.setTargetPosition(-810);

         } else if (key == 4) {
             mainArm.setTargetPosition(0);
             mainArm.setPower(.1);
         }
         if (gamepad1.left_bumper) {
             mainArm.setTargetPosition(-412);

         } else if (gamepad1.right_bumper) {
             mainArm.setTargetPosition(-412);
         }
         mainArm.setPower(.2);
     }

     public void drive() {

         double r = Math.hypot(-gamepad1.left_stick_x, gamepad1.left_stick_y);
         double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
         double rightX = gamepad1.right_stick_x;
         final double v1 = r * Math.cos(robotAngle) + rightX;
         final double v2 = r * Math.sin(robotAngle) - rightX;
         final double v3 = r * Math.sin(robotAngle) + rightX;
         final double v4 = r * Math.cos(robotAngle) - rightX;

         frontLeftMotor.setPower(v1 / 2);
         frontRightMotor.setPower(v2 / 2);
         backLeftMotor.setPower(v3 / 2);
         backRightMotor.setPower(v4 / 2);
     }

     //a=rotation
     //x=forward/backward
     //y=left/right
     //t = time in miliseconds

     public void ComputeMotorPowers(double vx, double vy, double a, long t) {
         final double r = 2;
         final double lx = 15;
         final double ly = 10;

         double w1 = (1 / r) * (vx - vy - (lx + ly) * a);
         double w2 = (1 / r) * (vx + vy + (lx + ly) * a);
         double w3 = (1 / r) * (vx + vy - (lx + ly) * a);
         double w4 = (1 / r) * (vx - vy + (lx + ly) * a);

         frontLeftMotor.setPower(w1);
         frontRightMotor.setPower(w2);
         backLeftMotor.setPower(w3);
         backRightMotor.setPower(w4);

         sleep(t);
     }

     public static class UltimateGoalPipeline extends OpenCvPipeline {

         public enum ringPosition {
             FOUR,
             ONE,
             NONE
         }

         //defining colors
         static final Scalar BLUE = new Scalar(0, 0, 255);
         static final Scalar GREEN = new Scalar(0, 225, 0);

         static final Point REGION1_TOPLEFT_ANCHOR_POINT = new Point(50, 200);

         static final int REGION_WIDTH = 45;
         static final int REGION_HEIGHT = 35;

         final int FOUR_RING_THRESHOLD = 150;
         final int ONE_RING_THRESHOLD = 135;

         Point region1_pointA = new Point(
                 REGION1_TOPLEFT_ANCHOR_POINT.x,
                 REGION1_TOPLEFT_ANCHOR_POINT.y
         );
         Point region1_pointB = new Point(
                 REGION1_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
                 REGION1_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT
         );

         Mat region1_Cb;
         Mat YCrCb = new Mat();
         Mat Cb = new Mat();
         int avgl;

         public ringPosition position = ringPosition.FOUR;
         public ringPosition count = ringPosition.FOUR;


         void inputToCb(Mat input) {
             Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
             Core.extractChannel(YCrCb, Cb, 1);
         }

         @Override
         public void init(Mat firstFrame) {
             inputToCb(firstFrame);

             region1_Cb = Cb.submat(new Rect(region1_pointA, region1_pointB));
         }

         @Override
         public Mat processFrame(Mat input) {
             inputToCb(input);

             avgl = (int) Core.mean(region1_Cb).val[0];

             Imgproc.rectangle(
                     input,
                     region1_pointA,
                     region1_pointB,
                     BLUE,
                     2
             );

             position = ringPosition.FOUR;
             count = ringPosition.FOUR;

             if (avgl > FOUR_RING_THRESHOLD) {
                 position = ringPosition.FOUR;
             } else if (avgl > ONE_RING_THRESHOLD) {
                 position = ringPosition.ONE;
             } else {
                 position = ringPosition.NONE;
             }

             Imgproc.rectangle(
                     input,
                     region1_pointA,
                     region1_pointB,
                     GREEN,
                     -1
             );

             return input;
         }

         public int getAnalysis() {
             return avgl;
         }
     }

     public void raiseWobbleArm() {
         wobbleArm.setTargetPosition(raisedwobbleArm);
         wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         wobbleArm.setPower(.2);
     }

     public void lowerWobbleArm() {
         wobbleArm.setTargetPosition(loweredwobbleArm);
         wobbleArm.setMode(DcMotor.RunMode.RUN_TO_POSITION);
         wobbleArm.setPower(.2);
     }
 }

         //public void armAngle(){
             //int armChange = 0;
             //int jointChange = 0;

        //if (gamepad1.y) {
            //armChange = -308;
            //jointChange = 0;
        //} else if (gamepad1.x) {
            //armChange = -546;
            //jointChange = 134;
        //} else if (gamepad1.a) {
            //armChange = -806;
            //jointChange = 324;
        //} else if (gamepad1.b) {
            //armChange = 0;
            //jointChange = 0;
        //}
        //if (gamepad1.left_bumper) {
            //armChange = 403;
            //jointChange = 162;
        //}

        //addTargetPositions(armChange, jointChange);

        //setting speed
        //mainArm.setPower(0.2);
        //armJoint.setPower(0.5);
    //}

    //public void addTargetPositions(int armChange, int jointChange){
        //mainArm.setTargetPosition(armChange);
        //armJoint.setTargetPosition(jointChange);
    //}
//}
