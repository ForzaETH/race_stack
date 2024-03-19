# Checklist for Time Trials

## Jobs

- Pilot: <student 1>
- Mission Control: <student 2, 3, 4>
- Orbiters: <students 5, 6, 7, 8>

## Precheck
0. Use a fully charged battery and turn on the car
1. Check all walkie-talkie batteries
2. Check if you are in the correct branch (`master`) on the car.
3. `catkin build` and `sauce`
4. No wires in the way, battery is firmly positioned with tape
5. Wheel nuts tightened and pin checked
6. Walkie-Talkie connection is established between mission control [Localization_MissionControl] and [Pilot].
7. Make sure [Pilot] knows the normal and overtaking sectors of the track, as well as the raceline
8. [Orbiters] are equidistant along the track
9. [MissionControl] prep a clock to countdown the time remaining

## First Heat
0. Launch `roscore` and `base_system` on the car
1. Locally, launch `pit_rvizX` and `pitX rqt`.
2. [MissionControl] Check all `rqt` dynamic reconfigure parameters are good.
3. Launch time trials:  

  ```bash
  roslaunch stack_master time_trials.launch racecar_version:=<VERSION> LU_table:=<LU_table_name>
  ```
4. [MisisonControl] Check if localization is good. If needed, correct the position either telling the [Pilot] to localize or with the pose estimation button in RViz.
5. When all MissionControl parameters are OK, [Localization_MissionControl] gives a thumbs-up. When [Pilot] sees this, start autonomous driving.
6. Ready check: "pit ready" --> "orbiter ready" --> "driver ready"

## Second Heat
0. Decide a strategy, if needed.
1. Replace the battery and ensure it is secured.
2. Repeat [the checklist above](#first-heat).

## Procedure in case of a Crash

0. The two closest [Orbiters] run to the crash site.
1. One student raises the barriers while the [Driver] drives the car back onto the track.
2. [Driver] drives manually around the track while the barriers are repaired.
3. When localization is ready, [Localization_MissionControl] informs the [Driver].
4. The driver must return to the position of the crash.
5. When the driver receives the signal from the referees, resume autonomous driving. 

---
[Go back to the checklist index](./README.md)