# Checklist for head-to-head phase
# F1TENTH autonomous grand prix (ICRA'23 London) - ForzaETH
 
## Roles
- [Control_MissionControl] @ltognoni
- [Localization_MissionControl] @tialim
- [MissionControl] @eghignone
- [Pilot] @bastuckn
- [Orbiters] as many as possible

## Procedure 
### Precheck [MissionControl]:
0. Use a fully charged battery and turn on the car
1. Check all walkie-talkie batteries
2. Check if you are in the correct branch (`master`) on the car.
3. `catkin build` and `sauce`
4. No wires in the way, battery is firmly positioned with tape
5. Wheel nuts tightened and pin checked
6. Walkie-Talkie connection is established between mission control [Localization_MissionControl] and [Pilot].
7. Make sure [Pilot] knows the normal and overtaking sectors of the track, as well as the raceline
8. [Orbiters] are equidistant along the track
9. [Orbiter] get ready to countdown laps into walkie-talkie

### Strategy
- If we don't expect to overtake, put trailing distance higher (eg 2.0)
- If opponent is very slow (30% laptime slower) also higher trailing distance
- Otherwise keep 1.5m trailing distance
- Set `ratio_to_glob_path` in `dynamic_tracker_server` to expected opponent speed

### Head To Head
0. Launch `roscore` and `base_system` on the car
1. Locally, launch `pit_rvizX` and `pitX rqt`.
2. [MissionControl] Check all `rqt` dynamic reconfigure parameters are good.
3. Launch head-to-head:
  ```bash
  roslaunch stack_master headtohead.launch racecar_version:=<VERSION> LU_table:=<LU_table_name> overtake_mode:=spliner od_mode:=sami
  ```
4. Check that the `Perception Detect` and `Perception Tracking` printed out `Ready` in the terminal
5. **Check that `Perception Detect noMemorymode` is set!**  
6. [MisisonControl] Check if localization is good. If needed, correct the position either telling the [Pilot] to localize or with the pose estimation button in RViz.
7. When all MissionControl parameters are OK, [Localization_MissionControl] gives a thumbs-up. When [Pilot] sees this, start autonomous driving.
8. Ready check: "pit ready" --> "orbiter ready" --> "driver ready"

## Procedure in case of a Crash

0. The two closest [Orbiters] run to the crash site.
1. One student raises the barriers while the [Driver] drives the car back onto the track.
2. [Driver] drives manually around the track while the barriers are repaired.
3. When localization is ready, [Localization_MissionControl] informs the [Driver].
4. The driver must return to the position of the crash.
5. When the driver receives the signal from the referees, resume autonomous driving.
6. Check-Uncheck the `force_GBTRACK` option in `dynamic_statemachine_server` to ensure we restart the race in `GB_FREE` mode