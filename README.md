# Beta2024 (aka Swift)
Team 1218's new robot code repository for FRC 2024 season, Crescendo:
* Based on the initial 3.0.0 release of Team 6328's AdvantageKit - AdvancedSwerveDriveProject code, distributed here:
    * https://github.com/Mechanical-Advantage/AdvantageKit/releases

## Overview
This year, we made a LOT of changes from last year in terms of software. All software features of the robot will be listed below, but here I will summarize the significant jumps from last year. A top priority this year was truly dialing in our swerve drive and autos. This meant implementing PathPlanner and effective vision, neither of which we had last year. Another large priotity was implementing logging and simulation through the AdvantageKit framework and AdvantageScope logging software. In 2022, we had our own logging solution; in 2023, we did not log anything at all; this year's implementation has been far easier and more effective than anything else we have used. The following are some of the unique features we have implemented:
* NavX implementation for AdvantageKit
* PhotonVision camera integration with the AdvantageKit IO architecture
* Accurate shooting using purely pose estimation

## Swerve
Our swerve code this year is a big jump from our last season. Instead of writing our own swerve code, we adopted another team's and have put an emphasis on accurate pose estimation, effective logging and simulation, and path-planned autos.
* This swerve is built on Team 6328's "Advanced Swerve" template, with many of our own upgrades and customizations for our needs.
* Physically built with SDS MK4i L2 swerve modules, completely driven by Kraken motors.
* Features an integration of the NavX gyro with AdvantageKit along with an increased polling rate (250hz).
* Includes simulation and replay functionality for the swerve drive along with pose estimates from odometry and vision.
* Custom general-use drive commands that allow for driving to specified positions, aiming at targets, and limelight alignment.
* Integrates PathPlanner for autonomous mode for modular and consistent routines.

## Vision
For Crescendo, we wanted to try something new with our vision system. Last year, we did not make use of the AprilTags on the field but later realized how big of an asset they can. This year, we have implemented a vision system to get the most accurate and precise pose estimation possible, with the following additions:
* Four Arducam cameras, each of which is connected to its own Orange Pi 5 coprocessor running PhotonVision.
* Custom camera code that implements AdvantageKit's "IO" architecture pattern to permit logging and reply functionalty for our system.
* Implementation of Limelight for note-detection for intake tracking
* We do not aim directly at the AprilTags to shoot, but rather we purely use our vision pose estimate for aiming and aligning.
* Well-tuned standard devations for the kalman filter.

## Arm, Shooter, and Intake
Our arm/pivot was the main mechanism of our robot this year. The idea was that this would house pretty much everything related to scoring notes, for speaker, amp, and trap. We were not even shooting from range at our first competition due to a lack of note stability, and at SCH our shooter was inconsistent at best. However, through continuous small improvements, by the end of the season we were making over 90% of our shots into the speaker when shooting from the back of the stage and forward. We attribute this to the fact that our system applied a large amount of spin along with accurate pose estimations and a well-tuned shooting-curve. The arm as a whole included the following software features:
* Motion-profiled feedback control of the arm mechanism through two Falcon 500 motors and motion magic from the Phoenix 6 API.
* Arm position measured by a CANCoder which allowed us to implement absolute position for feedback control.
* Interpolating map to get arm angles for shooting from different positions on the field.
* Shooter wheels with differential frictions driven by two Kraken motors with precise velocity control.
* Automatic shooter velocity adjustment for passing and amp scoring based on our arm angle and field position.
* LED indicator and controller rumble to indicate to the driver and operator that the intake possesses a note.
* Small pivoting bar in that guides notes into the trap through manual control from the operator.

## Climber
The climber we decided to use was made from carbon-fiber boat hooks, which allowed us to get very high up on the chain in order to trap with our arm. However, because the boathooks tended to slip, finding a sensing solution was difficult. We tried magnetic limit switches, potentiometers, string potentiometers, and motor encoders throughout the course of the season, but none worked well enough with our design to be consistent so we put everything in the hands of our operator (who did an amazing job with pretty much zero software assistance on the climb)! The climbers specifically included the following:
* Each controlled by a simple voltage output on Falcon 500 motors.
* Dynamic controls for climber adjustment with the joystick (up moves both up, down moves both down, left only moves the left, right only the right, and values between the axis will move both at differential values)

## Shooting and Trapping in Action
Below is what I would consider to be a good showcase of what our code allowed us to do in teleop by the end of the season:
https://github.com/VulcanRobotics/Beta2024/assets/58490916/7c328b0c-fe46-4b72-b677-d904e0d45e66
