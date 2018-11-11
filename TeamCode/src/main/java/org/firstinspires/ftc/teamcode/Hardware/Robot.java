package org.firstinspires.ftc.teamcode.Hardware;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.Recording.InputManager;

public class Robot{

    public DriveTrain drive = new DriveTrain();
    public MineralSystem mineralSystem = new MineralSystem();
    //public Gyro gyro = new Gyro();
    public InputManager inputManager = new InputManager();
    public TeamMarkerer tm = new TeamMarkerer();
    public ComputerVision computerVision = new ComputerVision();


    public Robot() {

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry, DriveTrain.DriveTypes type){
        this.initialize(hardwareMap, telemetry, type);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry, DriveTrain.DriveTypes type) {

        drive.init(hardwareMap, telemetry, type);
        mineralSystem.init(hardwareMap, telemetry);
        //gyro.init(hardwareMap,telemetry);
        inputManager.init(hardwareMap, telemetry);
        tm.init(hardwareMap, telemetry);
        //computerVision.init(hardwareMap);


    }

}
