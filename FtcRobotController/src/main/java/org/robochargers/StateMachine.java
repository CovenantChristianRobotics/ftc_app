package org.robochargers;

/**
 * Created by vanbaren on 10/31/15.
 */
public class StateMachine {

    public enum StateConditional {
        TRUE, FALSE, DELAY,
        WAIT_MLEFT_START, WAIT_MRIGHT_START, WAIT_MBOTH_START, WAIT_MEITHER_START,
        WAIT_MLEFT_STOP, WAIT_MRIGHT_STOP, WAIT_MBOTH_STOP, WAIT_MEITHER_STOP,
        DONE
    }

    public enum StateAction {
        PASS, DELAY_START,
        MLEFT_TARGET_POSITION, MRIGHT_TARGET_POSITION,
        MLEFT_POWER, MRIGHT_POWER, MBOTH_POWER
    }

    private int thisState;
    private int nextState;
    private StateConditional condition;
    private StateAction action;
    private long intParam;
    private double realParam;

    private long delayStart;

    public StateMachine(int thisState, int nextState,
                        StateConditional condition, StateAction action,
                        long intParam, double realParam) {
        this.thisState = thisState;
        this.nextState = nextState;
        this.condition = condition;
        this.action = action;
        this.intParam = intParam;
        this.realParam = realParam;
        this.delayStart = 0;
    }

    public int doit() {
        switch (action) {
            case PASS:
                break;
            case DELAY_START:
                delayStart = System.currentTimeMillis();
                break;
            case MLEFT_TARGET_POSITION:
                break;
            case MRIGHT_TARGET_POSITION:
                break;
            case MLEFT_POWER:
                break;
            case MRIGHT_POWER:
                break;
            case MBOTH_POWER:
                break;
        }

        switch (condition) {
            case TRUE:
                return nextState;
            case FALSE:
                break;
            case DELAY:
                if ((System.currentTimeMillis() - delayStart) >= intParam) {
                    delayStart = 0;
                    return nextState;
                }
                break;
            case WAIT_MLEFT_START:
                break;
            case WAIT_MRIGHT_START:
                break;
            case WAIT_MBOTH_START:
                break;
            case WAIT_MEITHER_START:
                break;
            case WAIT_MLEFT_STOP:
                break;
            case WAIT_MRIGHT_STOP:
                break;
            case WAIT_MBOTH_STOP:
                break;
            case WAIT_MEITHER_STOP:
                break;
            case DONE:
                break;
        }
        return thisState;
    }
}