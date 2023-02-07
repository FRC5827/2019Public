package org.usfirst.frc.team5827.robot.dashboard_manager;

/**
 * Classes that output to the SmartDashboard
 * should implement this interface, and tell the
 * SmartDashboardManager that they exist.
 */

 public interface DashboardOutputter
 {
     public void updateDashboard();
     public void initializeDashboard();
 }