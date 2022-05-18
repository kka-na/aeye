# Autonomous License Project - VACS
Code for the **Inha University autonomous license project**.

This project is divided into three separate modules which are linked via ROS.
<details>
    <summary><b>System Diagram</b></summary>
  
![System Diagram](http://165.246.39.210/public/vacs_diagram.png)
  </details>

## Milestones 👷
<details>
  <summary><b>Hardware Fixes</b></summary>
  
- [x] SCC Radar separation (Radar module must be separated from vehicle CAN. Radar -> Cable Block -> Vehicle Can)
- [ ] Speaker installation (Speakers must be installed in a location where volume or on/off control is not possible)  
</details>
<details>
  <summary><b>Software Fixes</b></summary>
  
- [ ] BSD distance adjustments (Front and back)
- [ ] Limit lane change conditions (Disable lane change when road curvature is above threshold)
- [ ] Driver TOR (take over request) (TOR when two wheels depart lane)
- [ ] AEB distance adjustments (ACC distance should be fixed to level 4)
- [ ] Enable autonomous mode only when lane lines are visible
- [ ] TOR logging + Fix delay
- [ ] TOR visual + auditory alert 
</details>

### Completed Tasks (excluding paper work💦)
1. LiDAR BSD
2. Fix CAN Error
3. E-STOP button
4. Merge CAN modules
5. SCC speed and mode change
6. Turn off driver monitoring
7. Turn off UI
8. Error checking (LiDAR, Camera, INS, RTK)
9. VACS (UI Module)
10. HW Installation (Lattepanda, Nuvo, E Stop, Light)
11. CAN (RPM, Steering, Gear)
12. Logging

## Contributors
- [Kka-na](https://github.com/kka-na)
- [Jang-KJ](https://github.com/maru-jang)
