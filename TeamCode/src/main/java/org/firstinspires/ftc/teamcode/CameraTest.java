/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;


/**
 This program tests our EasyOpenCV Pipeline that gets the location of the biggest contour on the field
 *It includes all the neccessary bones for the camera to function.
 * At the bottom, it calls pipeline.getLocation() in a telemetry line
 */

@TeleOp(name="camera test")
@Disabled

public class CameraTest extends LinearOpMode {
    OpenCvCamera webcam;


    @Override
    public void runOpMode() {
        //Finding & Defining webcam, hardwaremap;
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        webcam = OpenCvCameraFactory.getInstance().createWebcam(hardwareMap.get(WebcamName.class, "Camera"), cameraMonitorViewId);
        int isPark = 1;
        CubeVisionPipeline pipeline = new CubeVisionPipeline(telemetry, 0,1);

        webcam.setPipeline(pipeline);

        AprilTagProcessor TagProcess;
        TagProcess = new AprilTagProcessor.Builder()
                .setDrawTagID(true)
                .setDrawTagOutline(true)
                .setDrawAxes(true)
                .setDrawCubeProjection(false)
                .build();

        VisionPortal.Builder VisionBuilder;
        VisionPortal AprilPortal;

        // Create a new VisionPortal Builder object.
        VisionBuilder = new VisionPortal.Builder();

        // Specify the camera to be used for this VisionPortal.
        VisionBuilder.setCamera(hardwareMap.get(WebcamName.class, "Camera"));      // Other choices are: RC phone camera and "switchable camera name".

        // Add the AprilTag Processor to the VisionPortal Builder.
        VisionBuilder.addProcessor(TagProcess);       // An added Processor is enabled by default.


        VisionBuilder.setCameraResolution(new Size(320, 240));  // Each resolution, for each camera model, needs calibration values for good pose estimation.
        VisionBuilder.setStreamFormat(VisionPortal.StreamFormat.YUY2);  // MJPEG format uses less bandwidth than the default YUY2.
        VisionBuilder.enableLiveView(true);    // Enable LiveView (RC preview).
        VisionBuilder.setAutoStopLiveView(true);     // Automatically stop LiveView (RC preview) when all vision processors are disabled.

    // Create a VisionPortal by calling build()
        AprilPortal = VisionBuilder.build();
        //AprilPortal.setProcessorEnabled(TagProcess, false);
        AprilPortal.stopStreaming();

        telemetry.addData("Status", "Initialized");
        telemetry.update();



        webcam.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener()
        {
            @Override
            public void onOpened()
            {
                webcam.startStreaming( 320, 240, OpenCvCameraRotation.UPRIGHT); //320, 240
            }

            @Override
            public void onError(int errorCode) {
                telemetry.addData("ERROR",errorCode);
                telemetry.update();
            }
        });



        waitForStart();
        webcam.closeCameraDevice();

        while (opModeIsActive()) {

            boolean a = gamepad1.a;
            telemetry.addData("Size ", pipeline.getSide());
            telemetry.addData("Rectangle-X", pipeline.getrectX());
            telemetry.update();

        }
    }
}
