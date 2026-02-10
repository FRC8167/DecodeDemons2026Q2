package org.firstinspires.ftc.teamcode.Cogintilities;

import java.util.Arrays;
import java.util.List;

public class VariousMath {
    private VariousMath() {}

    static public double median(double[] numArray) {
        // Source - https://stackoverflow.com/a
        // Posted by lynnyi, modified by community. See post 'Timeline' for change history
        // Retrieved 2026-01-13, License - CC BY-SA 3.0

        Arrays.sort(numArray);
        double median;
        if (numArray.length % 2 == 0)
            median = (numArray[numArray.length/2] + numArray[numArray.length/2 - 1])/2;
        else
            median = numArray[numArray.length/2];
        return median;
    }

    static public double median(List<Double> numList) {
        double[] array = numList.stream()
                .mapToDouble(Double::doubleValue)
                .toArray();
        return median(array);
    }

    static public double elapsedSecondsFromAbsoluteNano(long nanoTime) {
        long relativeNanos = System.nanoTime()-nanoTime;
        return relativeNanos/1000000000.0;
    }


}
