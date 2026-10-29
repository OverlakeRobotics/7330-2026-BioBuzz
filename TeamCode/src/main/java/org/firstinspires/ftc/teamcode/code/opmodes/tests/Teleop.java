/* Copyright (c) 2021 FIRST. All rights reserved.
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

package org.firstinspires.ftc.teamcode.code.opmodes.tests;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagSingleDetection;

import java.util.List;

/*
 * This file contains an example of a Linear "OpMode".
 * An OpMode is a 'program' that runs in either the autonomous or the teleop period of an FTC match.
 * The names of OpModes appear on the menu of the FTC Driver Station.
 * When a selection is made from the menu, the corresponding OpMode is executed.
 *
 * This particular OpMode illustrates driving a 4-motor Omni-Directional (or Holonomic) robot.
 * This code will work with either a Mecanum-Drive or an X-Drive train.
 * Both of these drives are illustrated at https://gm0.org/en/latest/docs/robot-design/drivetrains/holonomic.html
 * Note that a Mecanum drive must display an X roller-pattern when viewed from above.
 *
 * Also note that it is critical to set the correct rotation direction for each motor.  See details below.
 *
 * Holonomic drives provide the ability for the robot to move in three axes (directions) simultaneously.
 * Each motion axis is controlled by one Joystick axis.
 *
 * 1) Axial:    Driving forward and backward               Left-joystick Forward/Backward
 * 2) Lateral:  Strafing right and left                     Left-joystick Right and Left
 * 3) Yaw:      Rotating Clockwise and counter clockwise    Right-joystick Right and Left
 *
 * This code is written assuming that the right-side motors need to be reversed for the robot to drive forward.
 * When you first test your robot, if it moves backward when you push the left stick forward, then you must flip
 * the direction of all 4 motors (see code below).
 *
 * Use Android Studio to Copy this Class, and Paste it into your team's code folder with a new name.
 * Remove or comment out the @Disabled line to add this OpMode to the Driver Station OpMode list
 */
@Disabled
@TeleOp(name="Telop Test", group="Linear OpMode")

public class Teleop extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private static final boolean USE_CAMERA = true;
    private DcMotorEx frontLeft = null;
    private DcMotorEx backLeft= null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backRight = null;

    private AprilTagProcessor aprilTag = null;
    private VisionPortal visionPortal = null;
    private boolean streaming = true;
    private boolean previousBumper = false;


    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft= hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");

        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward.
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);
        //initVision();




        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        // Wait for the game to start (driver presses START)
        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();


        // run until the end of the match (driver presses STOP)
        while (opModeIsActive()) {

            double max;

            // POV Mode uses left joystick to go forward & strafe, and right joystick to rotate.
            double x = -gamepad1.left_stick_y;
            double y = gamepad1.left_stick_x;
            double yaw = gamepad1.right_stick_x;

            if (Math.hypot(x, y) < 0.1) {
                x = 0;
                y = 0;
            } else {
                double magnitude = Math.hypot(x, y);
                double scaledMagnitude = Math.pow(magnitude, 3);
                x /= magnitude;
                y /= magnitude;

                x *= scaledMagnitude;
                y *= scaledMagnitude;
            }

            double axial = x;  // Note: pushing stick forward gives negative value
            double lateral = y;
            yaw *= 0.7;
            lateral *= 1.1;

            // Combine the joystick requests for each axis-motion to determine each wheel's power.
            // Set up a variable for each drive wheel to save the power level for telemetry.
            double frontLeftPower = axial + lateral + yaw;
            double frontRightPower = axial - lateral - yaw;
            double backLeftPower = axial - lateral + yaw;
            double backRightPower = axial + lateral - yaw;

            // Normalize the values so no wheel power exceeds 100%
            // This ensures that the robot maintains the desired motion.
            max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
            max = Math.max(max, Math.abs(backLeftPower));
            max = Math.max(max, Math.abs(backRightPower));

            if (max > 1.0) {
                frontLeftPower /= max;
                frontRightPower /= max;
                backLeftPower /= max;
                backRightPower /= max;
            }

            if (gamepad1.left_trigger_pressed) {
                frontLeftPower *= 0.5;
                frontRightPower *= 0.5;
                backLeftPower *= 0.5;
                backRightPower *= 0.5;
            }

            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);


            double average_power = (Math.abs(backLeftPower) + Math.abs(frontLeftPower) + Math.abs(backRightPower) + Math.abs(frontRightPower)) / 4.0;

            // This is test code:
            //
            // Uncomment the following code to test your motor directions.
            // Each button should make the corresponding motor run FORWARD.
            //   1) First get all the motors to take to correct positions on the robot
            //      by adjusting your Robot Configuration if necessary.
            //   2) Then make sure they run in the correct direction by modifying the
            //      the setDirection() calls above.
            // Once the correct motors move in the correct direction re-comment this code.

            /*
            frontLeftPower  = gamepad1.x ? 1.0 : 0.0;  // X gamepad
            backLeftPower   = gamepad1.a ? 1.0 : 0.0;  // A gamepad
            frontRightPower = gamepad1.y ? 1.0 : 0.0;  // Y gamepad
            backRightPower  = gamepad1.b ? 1.0 : 0.0;  // B gamepad
            */
//            List <AprilTagDetection> currentDetections = aprilTag.getDetections();
//            boolean targetFound = false;
//            for (AprilTagDetection detection : currentDetections) {
//                if (detection.ftcPose != null) {
//                    targetFound = true;
//                    double currentDistance = detection.ftcPose.range;
//                    if (currentDistance <=30) {
//                        aprilTag.setDecimation(3);
//                    }
//                    else if (currentDistance>30 && currentDistance<=65) {
//                        aprilTag.setDecimation(2);
//                    }
//                    else{
//                        aprilTag.setDecimation(1);
//                    }
//                }
//                if (!targetFound) {
//                    aprilTag.setDecimation(1);
//                    telemetry.addData("Status: ","Searching for tags...");
//                }
//            }
//
//
//
//            visionTelemetry();
//
//            telemetry.update();
//
//        }
//        visionPortal.close();
//    }
//    private void visionTelemetry () {
//
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//
//        for (AprilTagDetection detection : detections) {
//            if (detection instanceof AprilTagSingleDetection) {
//                AprilTagSingleDetection singleDet = (AprilTagSingleDetection) detection;
//                telemetry.addData("ID: ", singleDet.id);
//            }
//
//
//            if (detection.ftcPose!= null) {
//                telemetry.addData("X-axis: ", detection.ftcPose.x);
//                telemetry.addData("Y-axis: ", detection.ftcPose.y);
//                telemetry.addData("Yaw: ", detection.ftcPose.yaw);
//            }
//        }
//
//    }
//
//    private void initVision () {
//        aprilTag = new AprilTagProcessor.Builder().build();
//        if (USE_CAMERA) {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
//                    .addProcessor(aprilTag)
//                    .enableLiveView(true)
//                    .build();
//        }
//        else {
//            visionPortal = new VisionPortal.Builder()
//                    .setCamera(BuiltinCameraDirection.BACK)
//                    .enableLiveView(true)
//                    .addProcessor(aprilTag)
//                    .build();
        }
    }

}
