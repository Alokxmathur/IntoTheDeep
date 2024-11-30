package org.firstinspires.ftc.teamcode.robot.operations;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.game.Match;
import org.firstinspires.ftc.teamcode.robot.Robot;

import java.util.ArrayList;
import java.util.Date;

/**
 * Created by alokmathur on 10/29/17.
 */

public class OperationThread extends Thread {
    private final Object threadLock = new Object();
    private final String title;
    //stack of operationsQueue to perform
    private ArrayList<Operation> operationsQueue = new ArrayList<Operation>();
    private Robot robot;
    private Telemetry telemetry;

    public OperationThread(Robot robot, String title, Telemetry telemetry) {
        this.robot = robot;
        this.title = title + " Operation Thread";
        this.telemetry = telemetry;
        Match.log(title + " created");
    }

    public void run() {
        Match.log(title + " started");
        while (true) {
            synchronized (threadLock) {
                //if we have performed the operation successfully,
                // we need to remove our current operation
                if (this.operationsQueue.size() > 0) {
                    Operation operation = this.operationsQueue.get(0);
                    if (operation.getOperationIsBeingProcessed()) {
                        if (operation.isAborted()) {
                            Match.log(title + ": Aborted operation: " + operation.toString()
                                    + " at " + Match.getInstance().getElapsed()
                                    + " in " + (new Date().getTime() - operation.getStartTime().getTime())
                                    + " mSecs");
                        }
                        else if (operation.isComplete()) {
                            this.operationsQueue.remove(0);
                            Match.log(title + ": Completed operation: " + operation.toString()
                                    + " at " + Match.getInstance().getElapsed()
                                    + " in " + (new Date().getTime() - operation.getStartTime().getTime())
                                    + " mSecs");
                        }
                    }
                }
                if (this.operationsQueue.size() > 0) {
                    Operation operation = this.operationsQueue.get(0);
                    //if we haven't already started this operation, start it
                    if (!operation.getOperationIsBeingProcessed()) {
                        Match.log(title + ": Starting operation: " + operation.toString());
                        operation.setOperationBeingProcessed();
                        operation.startOperation();
                    }
                }
            }
            try {
                Thread.sleep(10);
            }
            catch (Throwable e) {}
        }
    }

    public void queueUpOperation(Operation operation) {
        synchronized (threadLock) {
            this.operationsQueue.add(operation);
        }
    }

    public void abort() {
        synchronized (threadLock) {
            //if we performing an operation - abort it
            if (this.operationsQueue.size() > 0) {
                Operation operation = this.operationsQueue.get(0);
                if (operation.getOperationIsBeingProcessed()) {
                    Match.log(title + ": Aborting operation: " + operation.getTitle());
                    operation.abortOperation();
                    operation.setAborted(true);
                }
            }
            this.operationsQueue.clear();
        }
    }

    public boolean hasEntries() {
        synchronized (threadLock) {
            return this.operationsQueue.size() > 0;
        }
    }
}
