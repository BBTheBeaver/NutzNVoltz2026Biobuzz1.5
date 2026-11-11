package org.firstinspires.ftc.teamcode.OpMode;
import static com.pedropathing.utils.Utils.clamp;
import static org.firstinspires.ftc.teamcode.Configs.Globals.Constants.*;

import com.pedropathing.api.PoseFactory;
import com.pedropathing.drivetrain.DrivePowers;
import com.pedropathing.follower.ManualDrive;
import com.pedropathing.ivy.Scheduler;
import com.pedropathing.math.Pose;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.pedropathing.controllers.PIDController;

import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;
import org.firstinspires.ftc.teamcode.Configs.Globals.PoseStorage;
import org.firstinspires.ftc.teamcode.Configs.Globals.ShooterTable;

import java.util.Objects;

@TeleOp(name = "Teleop", group = "StarterBot")
//@Disabled
public class Teleop extends OpMode {
    Robot robot;

    double headingError   = 0;
    double distanceToGoal = 0;
    boolean  headingLock    = false;
    double[] goalPose = GOAL_POSE_BLUE;

    double holdX, holdY;

    private final PIDController headingController = new PIDController(2.5, 0.0, 0.1);
    private final PIDController holdXController = new PIDController(0.08, 0.0, 0.01);
    private final PIDController holdYController = new PIDController(0.08, 0.0, 0.01);

    boolean intakeOn = false;
    boolean reverseOn = false;

    boolean left_flower = false;
    boolean right_flower = false;

    boolean isBlue;



    final String[] options = {"Blue", "Red"};
    int selectedOption = 0;
    boolean upLast = false;
    boolean downLast = false;

    boolean positionHold = false;

    private final PoseFactory mirroredFactory = PoseFactory.radians().mirrorX(70.75).mirrorY(70.75);








    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        Scheduler.reset();
        telemetry.addData("Status", "Initialized");
        //Driver 1: Intake, Heading lock, Drive
        //Driver 2: Shoot
    }

    @Override
    public void init_loop() {
        if (gamepad1.dpad_down && !downLast) selectedOption = (selectedOption + 1) % options.length;
        if (gamepad1.dpad_up   && !upLast)   selectedOption = (selectedOption - 1 + options.length) % options.length;
        upLast   = gamepad1.dpad_up;
        downLast = gamepad1.dpad_down;

        telemetry.addLine("=== ALLIANCE SELECT ===");
        for (int i = 0; i < options.length; i++) {
            telemetry.addData(i == selectedOption ? "> " : "  ", options[i]);
        }
        telemetry.addLine("DPAD up/down to switch, Start to begin");
        telemetry.update();
    }

    @Override
    public void start() {
        isBlue = Objects.equals(options[selectedOption], "Blue");
        if (isBlue) {
            goalPose = GOAL_POSE_BLUE;
        } else {
            goalPose = GOAL_POSE_RED;
        }

        if (PoseStorage.lastAutoPose != null) {
            robot.follower.setPose(PoseStorage.lastAutoPose);
        }

    }
    private Pose m(Pose pose) {
        if (isBlue) {
            return mirroredFactory.of(pose.x(), pose.y(), pose.heading());
        }
        return pose;
    }

    @Override
    public void loop() {
        robot.periodic();
        Scheduler.execute(); // runs anything scheduled — nothing's scheduled yet, but it's plugged in and ready

        robot.launcher.setVelocity(ShooterTable.speedFor(distanceToGoal));

        if(gamepad2.dpad_down){
            robot.follower.setPose(m(RESET_GARDEN));
        }
        if(gamepad2.dpad_up){
            robot.follower.setPose(m(RESET_TOP));
        }
        if(gamepad2.dpad_left|| gamepad1.dpad_right){
            robot.follower.setPose(m(RESET_FLOWER));
        }


        if (robot.follower.pose().y() > 75) {
            goalPose[1] = 97.0;
        } else {
            goalPose[1] = 47.0;
        }

        // ── RightBumper — toggle heading lock ──────────────────────────────────────
        if (gamepad1.rightBumperWasPressed()) {
            headingLock = !headingLock;
        }
        double turnPower;
        if(headingLock) {
            double dx;
            double dy;

            dx = goalPose[0] - robot.follower.pose().x();
            dy = goalPose[1] - robot.follower.pose().y();

            double kV = -0.75; // start here, tune sign/magnitude by watching behavior while strafing
            double denom = dx * dx + dy * dy;
            double headingVelFF = denom > 1e-6
                    ? (dx * robot.follower.velocity().vy - dy * robot.follower.velocity().vx) / denom
                    : 0;
            distanceToGoal = Math.hypot(dx, dy);
            headingError = normalizeAngle(Math.atan2(dy, dx) + Math.PI - robot.follower.pose().heading());
            headingError = normalizeAngle(Math.atan2(dy, dx) + Math.PI - robot.follower.pose().heading());
            if (Math.abs(headingError) < Math.toRadians(1)) {
                turnPower = 0;
            } else {
                turnPower = headingController.calculate(0, headingError) + kV * headingVelFF;
            }
        }else {
            turnPower = gamepad1.right_stick_x;
            //isBlue ? gamepad1.right_stick_x : gamepad1.right_stick_x;
        }

        double flip = isBlue ? 1 : -1;

        // Field-centric drive
        DrivePowers powers = ManualDrive.fieldCentric(
                flip * gamepad1.left_stick_y,
                flip * gamepad1.left_stick_x,
                turnPower,
                robot.follower.pose().heading()
        );
        robot.follower.manual(powers);



        // Intake toggle
        if (gamepad1.leftBumperWasPressed()) {
            intakeOn = !intakeOn;
            if (intakeOn) {
                reverseOn = false;
            }
        }
        // Reverse toggle
        if (gamepad1.leftTriggerWasPressed()) {
            reverseOn = !reverseOn;
            if (reverseOn) {
                intakeOn = false;
            }
        }

        if (intakeOn) {
            robot.intake.setPower(INTAKE_FORWARD);
            robot.transfer.setPower(INTAKE_FORWARD);
            //robot.leftIntakeServo.setPower(INTAKE_FORWARD);
            //robot.rightIntakeServo.setPower(INTAKE_FORWARD);
        } else if (reverseOn) {
            robot.intake.setPower(INTAKE_REVERSE);
            robot.transfer.setPower(INTAKE_REVERSE);
            //robot.leftIntakeServo.setPower(INTAKE_REVERSE);
            //robot.rightIntakeServo.setPower(INTAKE_REVERSE);
        } else {
            robot.intake.setPower(0);
            //robot.leftIntakeServo.setPower(0);
            //robot.rightIntakeServo.setPower(0);
        }

        if(gamepad1.rightStickButtonWasPressed()){
            left_flower = !left_flower;
            right_flower = !right_flower;
        }

        if(left_flower){
            robot.left_flower.setPosition(FLOWER_RAMP_DOWN);
        } else if (!left_flower) {
            robot.left_flower.setPosition(FLOWER_RAMP_UP);
        }

        if(right_flower){
            robot.right_flower.setPosition(FLOWER_RAMP_DOWN);
        } else if (!right_flower) {
            robot.right_flower.setPosition(FLOWER_RAMP_UP);
        }



        if (gamepad1.y) {
            robot.transfer.setPower(1);
            robot.gate.setPosition(GATE_OPEN);
            robot.intake.setPower(INTAKE_FORWARD);
        } else {
            robot.transfer.setPower(0);
            robot.gate.setPosition(GATE_CLOSE);
        }

        // Telemetry
        telemetry.addData("Shooter RPM: ", robot.launcher.getVelocity());
        telemetry.addData("Distance from goal: ", distanceToGoal);
        telemetry.addData("X Pose: ", robot.follower.pose().x());
        telemetry.addData("Y Pose: ", robot.follower.pose().y());
        telemetry.addData("Position Hold", positionHold);
        telemetry.addData("Triggers", "left (%.2f), right (%.2f)", gamepad1.left_trigger, gamepad1.right_trigger);
        telemetry.addData("Heading Error (deg)", Math.toDegrees(headingError));
        telemetry.addData("Heading", "%.1f°", Math.toDegrees(robot.follower.pose().heading()));
        telemetry.addData("Loop Hz", robot.getLoopTimeHz());
        telemetry.addLine("Dpad_Down for garden reset");
        telemetry.addLine("Dpad_Up for top reset");
        telemetry.addLine("Dpad_left or Dpad_right for flower reset");

        telemetry.update();
    }

    private static double normalizeAngle(double angle) {
        return Math.atan2(Math.sin(angle), Math.cos(angle));
    }

    private static double clamp(double v, double max) {
        return Math.max(-max, Math.min(max, v));
    }
}