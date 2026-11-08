package org.firstinspires.ftc.teamcode.config.core.util.hardware;

public class ToggleButton {
    private boolean prev = false, stored, m_justChanged;

    public ToggleButton(boolean defaultValue){
        this.stored = defaultValue;
        this.m_justChanged = false;
    }

    /**
     * Updates the current state of the button
     * @param cur The current state of the button pressed
     * @return True if the button was just pressed (Went from false to true this loop)
     */
    public boolean input(boolean cur){
        m_justChanged = cur && !prev;
        if(m_justChanged) stored = !stored;
        prev = cur;
        return m_justChanged;
    }
    public boolean getVal(){
        return stored;
    }

    public void setVal(boolean newVal){
        stored = newVal;
    }
    public boolean justChanged(){
        return m_justChanged;
    }
}