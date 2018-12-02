package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Recording.InputManager;

public class Robot{

    public DriveTrain drive = new DriveTrain();
    public MineralSystem mineralSystem = new MineralSystem();
    public Hanger hanger = new Hanger();
    public InputManager inputManager = new InputManager();
    public TeamMarkerer tm = new TeamMarkerer();


    public Robot() {

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, DriveTrain.DriveTypes type, boolean isAuto){
        this.initialize(hardwareMap, telemetry, type, isAuto);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry, DriveTrain.DriveTypes type, boolean isAuto) {

        drive.init(hardwareMap, telemetry, type, isAuto);
        mineralSystem.init(hardwareMap, telemetry);
        hanger.init(hardwareMap, telemetry);
        inputManager.init(hardwareMap, telemetry);
        tm.init(hardwareMap, telemetry);


    }

}
