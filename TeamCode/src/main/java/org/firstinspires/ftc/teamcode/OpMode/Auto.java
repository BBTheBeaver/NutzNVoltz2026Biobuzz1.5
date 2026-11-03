package org.firstinspires.ftc.teamcode.OpMode;

import com.pedropathing.ivy.Scheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.Configs.Autos.AutoRoutine;
import org.firstinspires.ftc.teamcode.Configs.Autos.FarAuto;
import org.firstinspires.ftc.teamcode.Configs.Autos.ParkAuto;
import org.firstinspires.ftc.teamcode.Configs.Autos.SoloAuto;
import org.firstinspires.ftc.teamcode.Configs.Globals.AutoConfig;
import org.firstinspires.ftc.teamcode.Configs.Globals.PoseStorage;
import org.firstinspires.ftc.teamcode.Configs.Globals.Robot;

@Autonomous(name = "Auto", group = "Autos")
public class Auto extends OpMode {

    private Robot robot;

    // Selection stages: each one only starts after the previous is confirmed
    private enum Stage { ALLIANCE, AUTO, PARK, READY }
    private Stage stage = Stage.ALLIANCE;

    private final String[] options = {"Blue", "Red"};
    private int selectedOption_Alliance = 0; // 0 = Blue, 1 = Red

    private final AutoRoutine[] red_autos = {
            new FarAuto(false),
            new SoloAuto(false),
            new ParkAuto(false)
            // Add more autos here
    };

    private final AutoRoutine[] blue_auto = {
            new FarAuto(true),
            new SoloAuto(true),
            new ParkAuto(true)
            // Add more autos here
    };

    private int selectedIndex = 0;
    private AutoRoutine selectedAuto;

    private final String[] parkOptions = {"Top", "Bottom"};
    private boolean parkTop = false; // false = bottom

    private AutoRoutine[] currentList() {
        return selectedOption_Alliance == 0 ? blue_auto : red_autos;
    }

    @Override
    public void init() {
        robot = new Robot(hardwareMap);
        PoseStorage.lastAutoPose = null;
        Scheduler.reset();
        selectedAuto = currentList()[selectedIndex];
    }

    @Override
    public void init_loop() {
        boolean up = gamepad1.dpadUpWasPressed();
        boolean down = gamepad1.dpadDownWasPressed();
        boolean a = gamepad1.aWasPressed();
        boolean b = gamepad1.bWasPressed();

        switch (stage) {
            case ALLIANCE:
                if (down) selectedOption_Alliance = (selectedOption_Alliance + 1) % options.length;
                if (up)   selectedOption_Alliance = (selectedOption_Alliance - 1 + options.length) % options.length;
                if (a) {
                    selectedIndex = 0;
                    selectedAuto = currentList()[selectedIndex];
                    stage = Stage.AUTO;
                }
                telemetry.addLine("=== ALLIANCE SELECT ===");
                for (int i = 0; i < options.length; i++) {
                    telemetry.addData(i == selectedOption_Alliance ? "> " : "  ", options[i]);
                }
                telemetry.addLine("DPAD up/down, A to confirm");
                break;

            case AUTO:
                AutoRoutine[] list = currentList();
                if (down) selectedIndex = (selectedIndex + 1) % list.length;
                if (up)   selectedIndex = (selectedIndex - 1 + list.length) % list.length;
                selectedAuto = list[selectedIndex];
                if (a) stage = Stage.PARK;      // park choice only starts now
                if (b) stage = Stage.ALLIANCE;
                telemetry.addLine("=== AUTO SELECT ===");
                telemetry.addData("Alliance", options[selectedOption_Alliance]);
                telemetry.addData("Auto", selectedAuto.getClass().getSimpleName());
                telemetry.addLine("DPAD up/down, A to confirm, B to go back");
                break;

            case PARK:
                if (down) parkTop = !parkTop;   // dpad down toggles, like alliance
                if (a) stage = Stage.READY;
                if (b) stage = Stage.AUTO;
                telemetry.addLine("=== PARK SELECT ===");
                telemetry.addData("Alliance", options[selectedOption_Alliance]);
                telemetry.addData("Auto", selectedAuto.getClass().getSimpleName());
                for (int i = 0; i < parkOptions.length; i++) {
                    boolean isSelected = (i == 0) == parkTop;
                    telemetry.addData(isSelected ? "> " : "  ", parkOptions[i]);
                }
                telemetry.addLine("DPAD down to switch, A to confirm, B to go back");
                break;

            case READY:
                if (b) stage = Stage.PARK;
                telemetry.addLine("=== READY ===");
                telemetry.addData("Alliance", options[selectedOption_Alliance]);
                telemetry.addData("Auto", selectedAuto.getClass().getSimpleName());
                telemetry.addData("Park", parkTop ? "TOP" : "BOTTOM");
                telemetry.addLine("Press START to run (B to edit)");
                break;
        }
        telemetry.update();
    }

    @Override
    public void start() {
        AutoConfig.parkTop = parkTop;       // must be set BEFORE build()
        robot.follower.setPose(selectedAuto.getStartPose());
        Scheduler.schedule(selectedAuto.build(robot));
    }

    @Override
    public void loop() {
        robot.periodic();
        Scheduler.execute();

        PoseStorage.save(robot.follower.pose());
    }

    @Override
    public void stop() {
        if (robot != null) {
            PoseStorage.save(robot.follower.pose());
        }
    }
}