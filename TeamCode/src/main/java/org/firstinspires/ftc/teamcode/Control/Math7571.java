package org.firstinspires.ftc.teamcode.Control;

public class Math7571 {

    public Math7571(){

    }

    public int wrapAround(int x, int lowerBound, int upperBound){

        int range_size = upperBound - lowerBound + 1;

        if (x < lowerBound)
            x += range_size * ((lowerBound - x) / range_size + 1);

        return lowerBound + (x - lowerBound) % range_size;

    }


}
