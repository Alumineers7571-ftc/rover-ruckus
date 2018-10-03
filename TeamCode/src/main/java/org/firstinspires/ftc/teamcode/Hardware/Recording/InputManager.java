package org.firstinspires.ftc.teamcode.Hardware.Recording;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.BaseHardware;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class InputManager extends BaseHardware {

    public DriveTrain dt = new DriveTrain();

    public BufferedWriter bufferedWriter;
    public FileWriter fileWriter;
    public FileReader fileReader;
    public BufferedReader bufferedReader;

    //File file;

    double[] leftPowerValues;
    double[] rightPowerValues;

    double ly, rx;

    int countLines, countReplays;

    public InputManager(){

    }

    public void setupRecording(File file){

        // Assume default encoding.
        fileWriter = null;

        try {
            fileWriter = new FileWriter(file);
        } catch (IOException e) {
            e.printStackTrace();
        }

        // Always wrap FileWriter in BufferedWriter.
        bufferedWriter = new BufferedWriter(fileWriter);

    }

    public void writeInputs(Gamepad gamepad1){

        try {
            bufferedWriter.write(gamepad1.left_stick_y + ";" + gamepad1.right_stick_x);
            bufferedWriter.newLine();
        } catch (IOException e) {
            e.printStackTrace();
        }

        dt.manualDrive(gamepad1);

    }

    public void setupPlayback(File file){
        String[] tempString;
        String line;

        leftPowerValues = new double[50000];
        rightPowerValues = new double[50000];

        try {

            fileReader = new FileReader(file);
            bufferedReader = new BufferedReader(fileReader);



            while ((line = bufferedReader.readLine()) != null) {

                tempString = line.split(";");

                ly = Double.parseDouble(tempString[0]);
                rx = Double.parseDouble(tempString[1]);

                leftPowerValues[countLines] = Range.clip(ly + rx, -1, 1);
                rightPowerValues[countLines] = Range.clip(ly - rx, -1, 1);

                countLines++;
            }

            bufferedReader.close();

            countReplays = 0;
        }
        catch(IOException ex) {
            ex.printStackTrace();
        }
    }

    public void replayInputs(){
        while(countReplays <= countLines) {
            dt.runMotorsSides(leftPowerValues[countReplays], rightPowerValues[countReplays]);
            countReplays++;
        }
    }

    public void stopAndReset() {

        try {

            bufferedReader.close();
            bufferedWriter.close();

        } catch (IOException e) {
            e.printStackTrace();
        }

    }

    public void init(HardwareMap hardwareMap, Telemetry telemetry) {
        this.initialize(hardwareMap, telemetry);
    }

    private void initialize(HardwareMap hardwareMap, Telemetry telemetry){

        dt.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK);


    }
}
