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

import androidx.annotation.NonNull;

import com.qualcomm.hardware.rev.RevHubOrientationOnRobot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.IMU;


import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.YawPitchRollAngles;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;



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

@Autonomous(name="Autonomous Test", group="Linear OpMode")
public class Autonomous_Methods extends LinearOpMode {

    // Declare OpMode members for each of the 4 motors.
    private static final boolean USE_CAMERA = true;


    private DcMotorEx frontLeft = null;
    private DcMotorEx backLeft = null;
    private DcMotorEx frontRight = null;
    private DcMotorEx backRight = null;
    private IMU imu = null;
    private VisionPortal visionPortal = null;
    private AprilTagProcessor aprilTag = null;

    @Override
    public void runOpMode() {

        // Initialize the hardware variables. Note that the strings used here must correspond
        // to the names assigned during the robot configuration step on the DS or RC devices.
        frontLeft = hardwareMap.get(DcMotorEx.class, "frontLeft");
        backLeft = hardwareMap.get(DcMotorEx.class, "backLeft");
        frontRight = hardwareMap.get(DcMotorEx.class, "frontRight");
        backRight = hardwareMap.get(DcMotorEx.class, "backRight");
        imu = hardwareMap.get(IMU.class, "imu");
        RevHubOrientationOnRobot.LogoFacingDirection logoDirection = RevHubOrientationOnRobot.LogoFacingDirection.FORWARD;
        RevHubOrientationOnRobot.UsbFacingDirection usbDirection = RevHubOrientationOnRobot.UsbFacingDirection.UP;
        RevHubOrientationOnRobot orientationOnRobot = new RevHubOrientationOnRobot(logoDirection, usbDirection);
        imu.initialize(new IMU.Parameters(orientationOnRobot));
        //initVision();

        /*
        // ########################################################################################
        // !!!            IMPORTANT Drive Information. Test your motor directions.            !!!!!
        // ########################################################################################
        // Most robots need the motors on one side to be reversed to drive forward.
        // The motor reversals shown here are for a "direct drive" robot (the wheels turn the same direction as the motor shaft)
        // If your robot has additional gear reductions or uses a right-angled drive, it's important to ensure
        // that your motors are turning in the correct direction.  So, start out with the reversals here, BUT
        // when you first test your robot, push the left joystick forward and observe the direction the wheels turn.
        // Reverse the direction (flip FORWARD <-> REVERSE ) of any wheel that runs backward
        // Keep testing until ALL the wheels move the robot forward when you push the left joystick forward. */
        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.FORWARD);

        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        telemetry.addData("Status", "Initialized");
//Velocity must be between 1000 to 3000. Anything outside this parameter will be either too fast or slow.
        waitForStart();
        resetEncode();

        drive(500,1000);
        IMUTurn(40);
        strafe(300,1000);
        diagonal_strafe(200,9);



        resetEncode();
    }

    private void drive(double centimeters, double velocity) {

        centimeters = ticks_to_cm(centimeters);
        resetEncode();
        while (opModeIsActive()) {
            double average = (Math.abs(backLeft.getCurrentPosition()) + Math.abs(frontLeft.getCurrentPosition()) + Math.abs(backRight.getCurrentPosition()) + Math.abs(frontRight.getCurrentPosition())) / 4.0;
            double averageVelocity = (Math.abs(backLeft.getVelocity()) + Math.abs(frontLeft.getVelocity()) + Math.abs(backRight.getVelocity()) + Math.abs(frontRight.getVelocity())) / 4.0;
            double progress = Math.abs(average) / Math.abs(centimeters);
            if (Math.abs(progress) >= 1.02) {

                break;
            }


            double yawCorrection = -getYaw() * 0.01;
            double current_velocity = velocity;
            if (Math.abs(progress) > 0.8) {
                double t = (progress - 0.8) / 0.2;
                t = Math.min(Math.max(t, 0), 1);
                current_velocity = velocity * (1 - t * t);
            }
            frontLeft.setVelocity(current_velocity - (yawCorrection * 0.5));
            backLeft.setVelocity(current_velocity - (yawCorrection * 0.5));
            backRight.setVelocity(current_velocity + (yawCorrection * 0.5));
            frontRight.setVelocity(current_velocity + (yawCorrection * 0.5));
            telemetryUpdate(
                    "Average: ", cm_to_ticks(average),
                    "Yaw: ", getYaw(),
                    "Progress: ", progress,
                    "Velocity: ", averageVelocity


            );

            idle();
        }

        resetEncode();

    }

    private void strafe(double centimeters, double velocity) {
        centimeters = ticks_to_cm(centimeters);
        resetEncode();
        int lfStart = frontLeft.getCurrentPosition();
        int lbStart = backLeft.getCurrentPosition();
        int rfStart = frontRight.getCurrentPosition();
        int rbStart = backRight.getCurrentPosition();

        while (opModeIsActive()) {
            int lfDelta = frontLeft.getCurrentPosition() - lfStart;
            int lbDelta = backLeft.getCurrentPosition() - lbStart;
            int rfDelta = frontRight.getCurrentPosition() - rfStart;
            int rbDelta = backRight.getCurrentPosition() - rbStart;
            double average = -(Math.abs(lbDelta) + Math.abs(lfDelta) - Math.abs(rfDelta) + Math.abs(rbDelta)) / 4.0;
            double averageVelocity = (Math.abs(backLeft.getVelocity()) + Math.abs(frontLeft.getVelocity()) + Math.abs(backRight.getVelocity()) + Math.abs(frontRight.getVelocity())) / 4.0;
            double progress = Math.abs(average) / Math.abs(centimeters);
            if (progress >= 1.02) {
                break;
            }
            double yawCorrection = -getYaw() * 0.02;
            double currentVelocity = velocity;
            if (Math.abs(progress) >= 0.8) {
                double t = (progress - 0.8) / 0.2;
                t = Math.min(Math.max(t, 0), 1);
                currentVelocity = velocity * (1 - t * t);
            }

            frontLeft.setVelocity(currentVelocity - yawCorrection);
            backRight.setVelocity(currentVelocity + yawCorrection);
            backLeft.setVelocity(-currentVelocity - yawCorrection);
            frontRight.setVelocity(-currentVelocity + yawCorrection);
            telemetryUpdate(
                    "Average: ", cm_to_ticks(average),
                    "Yaw: ", getYaw(),
                    "Progress: ", progress,
                    "Velocity: ", averageVelocity
            );
            idle();
        }

        resetEncode();

    }

    private void diagonal_strafe(double centimeters, double angles) {
        centimeters = ticks_to_cm(centimeters);
        double radians = Math.toRadians(angles);
        double strafe = Math.sin(radians);
        double forward = Math.cos(radians);
        double speed = 1500;
        //Finding tan via desired angles and normalizing them

        resetEncode();
        int lfStart = frontLeft.getCurrentPosition();
        int lbStart = backLeft.getCurrentPosition();
        int rfStart = frontRight.getCurrentPosition();
        int rbStart = backRight.getCurrentPosition();


        while (opModeIsActive()) {
            int lfDelta = frontLeft.getCurrentPosition() - lfStart;
            int lbDelta = backLeft.getCurrentPosition() - lbStart;
            int rfDelta = frontRight.getCurrentPosition() - rfStart;
            int rbDelta = backRight.getCurrentPosition() - rbStart;
            double strafing = (-lfDelta + rfDelta + lbDelta - rbDelta) / 4.0;
            double forwarding = (lfDelta + lbDelta + rbDelta + rfDelta) / 4.0;
            double actualAngle = Math.toDegrees(Math.atan2(strafing, forwarding));
            double actual_average = forwarding * forward + strafing * strafe;
            double progress = Math.abs(actual_average) / Math.abs(centimeters);
            double averageVelocity = (Math.abs(backLeft.getVelocity()) + Math.abs(frontLeft.getVelocity()) + Math.abs(backRight.getVelocity()) + Math.abs(frontRight.getVelocity())) / 4.0;

            if (Math.abs(progress) >= 1.02) {
                break;
            }
            double yaw = getYaw();
            double yawCorrection = -yaw * 0.01;
            double error = angleError(angles, yaw);
            double angleCorrection = error * 0.02;
            double correctedStrafe = strafe + angleCorrection;
            double max = Math.max(Math.abs(forward) + Math.abs(correctedStrafe), 1);
            double lf = (forward + correctedStrafe) / max;
            double lb = (forward - correctedStrafe) / max;
            double rf = (forward - correctedStrafe) / max;
            double rb = (forward + correctedStrafe) / max;
            double current_speed = speed;

            if (Math.abs(progress) > 0.8) {
                double t = (progress - 0.8) / 0.2;
                t = Math.min(Math.max(t, 0), 1);
                current_speed = speed * (1 - t * t);
            }
            frontLeft.setVelocity(lf * current_speed - yawCorrection);
            backLeft.setVelocity(lb * current_speed - yawCorrection);
            frontRight.setVelocity(rf * current_speed + yawCorrection);
            backRight.setVelocity(rb * current_speed + yawCorrection);
            telemetryUpdate(
                    "Current Angle: ", actualAngle,
                    "Average: ", cm_to_ticks(actual_average),
                    "Progress: ", Math.round(progress),
                    "Velocity: ", averageVelocity
            );

            idle();
        }

        resetEncode();
    }

    private void IMUTurn(double angles) {
        double yaw = getYaw();
        YawPitchRollAngles orientation;
        double turn_power;
        double error = angleError(angles, yaw);
        //Decrease to reduce speed. Originally was at a value of 200.
        double p = 50;
        while (Math.abs(error) >= 1 ) {
            double minimum_power = 0.02;
            double averageVelocity = (Math.abs(backLeft.getVelocity()) + Math.abs(frontLeft.getVelocity()) + Math.abs(backRight.getVelocity()) + Math.abs(frontRight.getVelocity())) / 4.0;
            orientation = imu.getRobotYawPitchRollAngles();
            yaw = orientation.getYaw(AngleUnit.DEGREES);
            error = angleError(angles, yaw);
            turn_power = error * p;
            if (Math.abs(turn_power) < minimum_power) {
                turn_power = Math.copySign(minimum_power, turn_power);
            }
            frontLeft.setVelocity(-turn_power);
            backLeft.setVelocity(-turn_power);
            frontRight.setVelocity(turn_power);
            backRight.setVelocity(turn_power);
            telemetryUpdate(
                    "Yaw: ", yaw,
                    "Turn Power: ", turn_power,
                    "Error: ", error,
                    "Velocity: ", averageVelocity
            );
            idle();
        }


        resetEncode();
    }

    private void telemetryUpdate(@NonNull Object... data) {
        //IMPORTANT NOTES BELOW
        /*There has to be an even amount of telemetry or else it will crash. For instance, this would work:
    telemetryUpdate(
                "Data: ", frontRight.getCurrentPosition(),    <----Comma is necessary if you have multiple lines of telemetry
                "Data: ", String.format("%7d, %7d", frontLeft.getCurrentPosition(),frontRight.getCurrentPosition())        <---No comma necessary if it is the last piece of telemetry
                );
    BUT, this:
    telemetryUpdate(
                "Data: ", String.format("%7d, %7d", backLeft.getCurrentPosition(),frontRight.getCurrentPosition()) <--No comma = crash
                "Data 2: backLeft.getCurrentPosition()",
                "Data 3: backLeft.getCurrentPosition()"
                );
         wouldn't work because it has an odd amount of telemetry.

       Remove the -1 from the i<data.length -1 section to allow telemetry to accept any odd amount of telemetry


         */
        for (int i = 0; i < data.length - 1; i += 2) {
            telemetry.addData(data[i].toString(), data[i + 1]);
        }

        telemetry.update();
    }

    private double angleError(double target, double current) {
        double error = target - current;
        while (error >= 180) error -= 360;
        while (error <= -180) error += 360;
        return error;

    }

    private double getYaw() {
        YawPitchRollAngles orientation = imu.getRobotYawPitchRollAngles();
        return orientation.getYaw(AngleUnit.DEGREES);
    }

    private void resetEncode() {
        setV(0);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        imu.resetYaw();
        //ensuring everything reset
        idle();
        sleep(500);

    }

    private void setV(double power) {
        frontLeft.setVelocity(power);
        backRight.setVelocity(power);
        backLeft.setVelocity(power);
        frontRight.setVelocity(power);
    }


    private double ticks_to_cm(double ticks) {
        double conversion_rate = 16.4541725781;
        return ticks * conversion_rate;

    }

    private double cm_to_ticks(double cm) {
        double conversion_rate = 16.4541725781;
        return cm / conversion_rate;
    }
//    private void vision () {
//
//        List<AprilTagDetection> detections = aprilTag.getDetections();
//
//        for (AprilTagDetection detection : detections) {
//            if (detection instanceof AprilTagSingleDetection) {
//                AprilTagSingleDetection singleDet= (AprilTagSingleDetection) detection;
//                telemetry.addData("ID: ", singleDet.id);
//            }
//
//
//            if (detection.ftcPose!= null) {
//                telemetry.addData("X-axis: ", detection.ftcPose.x);
//                telemetry.addData("Y-axis: ", detection.ftcPose.y);
//                telemetry.addData("Yaw: ", detection.ftcPose.yaw);
//
//
//            }
//        }
//    }
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
//                    .addProcessor(aprilTag)
//                    .build();
//        }
 //   }
}





