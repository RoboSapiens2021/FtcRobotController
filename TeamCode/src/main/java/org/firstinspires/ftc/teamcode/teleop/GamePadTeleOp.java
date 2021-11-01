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

package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.Robot;
import org.firstinspires.ftc.teamcode.motor.DriveTrain;
import org.firstinspires.ftc.teamcode.util.Logger;

@TeleOp(name = "GamePadTeleOp", group = "TeleOp")
/**
 * POV Mode uses left stick to go forward, and right stick to turn
 */
public class GamePadTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() {
        Robot robot = Robot.getInstance();
        robot.setOpMode(this);
        robot.init();

        Logger logger = Logger.getInstance();
        logger.debug("Gamepad1 User" + gamepad1.getUser());
        logger.debug("Gamepad2 User " + gamepad2.getUser());
        GameController gameController = new GameController(robot.getSpinner());

        // run until the end of the match (driver presses STOP)
        DriveTrain driveTrain = robot.configureDriveTrain();

        while (opModeIsActive()) {
            double[] motorPower = getMotorPower();
            driveTrain.setMotorPowers(motorPower[0], motorPower[1], motorPower[2], motorPower[3]);
            gameController.turnSpinner(gamepad1.right_trigger - gamepad1.left_trigger);
        }
    }

    private double[] getMotorPower() {
        double r = Math.hypot(gamepad1.left_stick_x, gamepad1.left_stick_y);
        double robotAngle = Math.atan2(-gamepad1.left_stick_y, gamepad1.left_stick_x) - (Math.PI / 4);
        double rightX = gamepad1.right_stick_x;

        double cosine = Math.cos(robotAngle);
        double sine = Math.sin(robotAngle);

        double[] motorPower = new double[4];
        motorPower[0] = r * cosine + rightX;// lf
        motorPower[1] = r * sine + rightX;//lr
        motorPower[2] = r * cosine - rightX;//rr
        motorPower[3] = r * sine - rightX;//rf

        return motorPower;
    }

//    private double[] getMotorPower() {
//        double y = -gamepad1.left_stick_y;
//        double x = gamepad1.left_stick_x;
//        double r = gamepad1.right_stick_x;
//
//        double max = Math.max(Math.abs(y), Math.max(Math.abs(x), Math.abs(r)));
//
//        double[] motorPower = new double[4];
//        motorPower[0] = (y + x + r) / max; // lf
//        motorPower[1] = (y - x + r) / max; // lr
//        motorPower[2] = (y + x - r) / max; // rr
//        motorPower[3] = (y - x - r) / max; // rf
//
//        return motorPower;
//    }

}



