package org.firstinspires.ftc.teamcode.Hardware.Recording;

import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.DriveTrain;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class InputManager {

    public DriveTrain dt = new DriveTrain();

    public BufferedWriter bufferedWriter;
    public FileWriter fileWriter;
    public FileReader fileReader;
    public BufferedReader bufferedReader;

    //File file;

    double[] leftFrontValues;
    double[] leftBackValues;
    double[] rightFrontValues;
    double[] rightBackValues;

    double ly, rx, lx;

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
            bufferedWriter.write(gamepad1.left_stick_y + ";" + gamepad1.right_stick_x + ";" + gamepad1.left_stick_x);
            bufferedWriter.newLine();
        } catch (IOException e) {
            e.printStackTrace();
        }

        dt.manualDrive(gamepad1);

    }

    public void setupPlayback(File file){
        String[] tempString;
        String line;

        leftFrontValues = new double[50000];
        rightFrontValues = new double[50000];

        try {

            fileReader = new FileReader(file);
            bufferedReader = new BufferedReader(fileReader);



            while ((line = bufferedReader.readLine()) != null) {

                tempString = line.split(";");

                ly = Double.parseDouble(tempString[0]);
                rx = Double.parseDouble(tempString[1]);
                lx = Double.parseDouble(tempString[2]);

                leftFrontValues[countLines] = Range.clip(ly + rx + lx, -1, 1);
                leftBackValues[countLines] = Range.clip(ly + rx - lx, -1, 1);
                rightFrontValues[countLines] = Range.clip(ly - rx - lx, -1, 1);
                rightBackValues[countLines] = Range.clip(ly - rx + lx, -1, 1);

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
            dt.runMotorsIndiv(leftFrontValues[countReplays], rightFrontValues[countReplays], leftBackValues[countReplays], rightBackValues[countReplays]);
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

        dt.init(hardwareMap, telemetry, DriveTrain.DriveTypes.TANK, false);


    }
}
