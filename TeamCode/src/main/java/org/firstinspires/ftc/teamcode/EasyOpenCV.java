package org.firstinspires.ftc.teamcode;

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


public class EasyOpenCV extends Gen1Auto{
    OpenCvInternalCamera phoneCam;
    UltimateGoalPipeline pipeline;

     @Override
    public void runOpMode() {
         int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
         phoneCam = OpenCvCameraFactory.getInstance().createInternalCamera(OpenCvInternalCamera.CameraDirection.BACK, cameraMonitorViewId);
         pipeline = new UltimateGoalPipeline();
         phoneCam.setPipeline(pipeline);

         phoneCam.setViewportRenderingPolicy(OpenCvCamera.ViewportRenderingPolicy.OPTIMIZE_VIEW);

         phoneCam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
             @Override
             public void onOpened() {
                 phoneCam.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT);
             }
         });


         waitForStart();

         while (opModeIsActive())
         {
             telemetry.addData("Analysis" , pipeline.getAnalysis());
             telemetry.addData("Position", pipeline.position);
             telemetry.update();

             sleep(50);

         }
     }

     public static class UltimateGoalPipeline extends OpenCvPipeline{

         public enum ringPosition{
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

         public volatile ringPosition position = ringPosition.FOUR;

         void inputToCb(Mat input){
             Imgproc.cvtColor(input, YCrCb, Imgproc.COLOR_RGB2YCrCb);
             Core.extractChannel(YCrCb, Cb, 1);
         }

         @Override
         public void init(Mat firstFrame){
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

             if (avgl > FOUR_RING_THRESHOLD) {
                 position = ringPosition.FOUR;
             }
             else if(avgl > ONE_RING_THRESHOLD) {
                 position = ringPosition.ONE;
             }
             else {
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
         public int getAnalysis(){
             return avgl;
         }


     }
}
