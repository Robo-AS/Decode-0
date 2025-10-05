package org.firstinspires.ftc.teamcode.programs.opmodes.auto.pedro;

import com.acmerobotics.dashboard.config.Config;

@Config
public class AutoPaths {
    private static AutoPaths instance = null;

    public static AutoPaths getInstance() {
        if (instance == null) instance = new AutoPaths();
        return instance;
    }


    public boolean LAUNCH_PRELOAD_COMPLETED = false;
    public boolean GATE_COMPLETED = false;
    public boolean GO_TO_INTAKE_COMPLETED = false;
    public boolean GET1_COMPLETED = false;
    public boolean THROW1_COMPLETED = false;
    public boolean GET2_COMPLETED = false;
    public boolean THROW2_COMPLETED = false;

    public void resetAll() {
        LAUNCH_PRELOAD_COMPLETED = false;
        GATE_COMPLETED = false;
        GO_TO_INTAKE_COMPLETED = false;
        GET1_COMPLETED = false;
        THROW1_COMPLETED = false;
        GET2_COMPLETED = false;
        THROW2_COMPLETED = false;
    }
}
