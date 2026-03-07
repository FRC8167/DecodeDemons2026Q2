package org.firstinspires.ftc.teamcode.Cogintilities.EricsCrap;

public class TimedTimer {
    private double initialTime;
    private double duration;

    private boolean frozen = false;
    private double frozenTime = Double.NaN;

    public TimedTimer(double timeInSeconds) {
        initialTime = System.currentTimeMillis();
        duration = timeInSeconds*1000;
    }

    public TimedTimer() {
        initialTime = System.currentTimeMillis();
        duration = 0;
    }

    public void startNewTimer(double timeInSeconds) {
        initialTime = System.currentTimeMillis();
        duration = timeInSeconds*1000;
        frozen = false;
    }

    public void restart() {
        initialTime = System.currentTimeMillis();
        frozen = false;
    }

    private double time() {
        if (!frozen)
            return System.currentTimeMillis();
        else return frozenTime;
    }

    public void freeze() {
        if (!frozen) {
            frozenTime = System.currentTimeMillis();
            frozen = true;
        }
    }

    public void unfreeze() {
        if (frozen) {
            initialTime += System.currentTimeMillis() - frozenTime;
            frozenTime = Double.NaN;
            frozen = false;
        }
    }




    public boolean isDone() {
        return (initialTime + duration <= time());
    }

    public double getRemainingTime() {
        return Math.max(0, (initialTime + duration) - time()) / 1000.0;
    }

    public double getProportionCompleted() {
        if (duration == 0) return -1;
        return (duration - (Math.max(0, (initialTime + duration) - time())))/duration;
    }

    public double getElapsedTime() {
        return (time() - initialTime) / 1000.0;
    }

    public boolean isFrozen() {
        return frozen;
    }

}
