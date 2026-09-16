# PacBot_eYantra-26
Current Info: for now in the pacbot task1b is pending refer the following steps if u don't want to continue at beginning 
for access of the file "pacbot_ws" need continue just before the Mujoco :"https://drive.google.com/file/d/1PnnKGHwHTSpqOSuAKoN21u45LB3KyvEe/view?usp=sharing"

STEPS:
# PacBot Task 1B — Current Status & Next Steps

## Current Status

The initial development of the Task 1B controller has been completed on the development PC.

### Completed

* Task 1B workspace prepared:
  `~/pacbot_ws/task1b`
* `task_1b.py` has been replaced with the current wall-following + PID controller.
* Original controller is backed up as:
  `task_1b_backup.py`
* Python syntax has been verified successfully:

  ```bash
  python3 -m py_compile task_1b.py
  ```
* Mosquitto MQTT broker is installed and running.
* `paho-mqtt` Python package is installed and working.

## Important Issue

The official MuJoCo launcher cannot run on the development PC.

Running:

```bash
./task_1b_launch
```

results in:

```text
Illegal instruction (core dumped)
```

Therefore, **do not spend time trying to run MuJoCo on that PC**.

The official `task_1b_launch` must not be modified, decompiled, or tampered with.

The next development and testing work must be done on the compatible PC where MuJoCo runs correctly.

---

# Current Controller

The current `task_1b.py` contains:

* MQTT sensor subscription
* Wall-following logic
* PID control
* Left/right wall selection
* Front obstacle detection
* Wheel velocity publishing
* Basic error handling

Current controller parameters:

```text
BASE_SPEED = 2.0
TARGET_WALL_DISTANCE = 0.20
KP = 4.0
KI = 0.05
KD = 0.25
MAX_CORRECTION = 1.5
INTEGRAL_LIMIT = 0.5
FRONT_THRESHOLD = 0.22
```

This is a **development/test version**, not the final tuned controller.

---

# NEXT STEPS — DO THESE ON THE COMPATIBLE PC

## 1. Copy `task_1b.py`

Copy the latest `task_1b.py` into:

```bash
~/pacbot_ws/task1b/
```

Make sure the folder contains:

```text
task_1b_launch
task_1b.py
lib/
meshes/
```

Then:

```bash
cd ~/pacbot_ws/task1b
python3 -m py_compile task_1b.py
```

There should be no output.

---

## 2. Check Mosquitto

Run:

```bash
sudo systemctl status mosquitto
```

It should show:

```text
Active: active (running)
```

If required:

```bash
sudo systemctl start mosquitto
```

Check MQTT Python support:

```bash
python3 -c "import paho.mqtt.client as mqtt; print('paho-mqtt OK')"
```

---

# 3. FIRST TEST — NORMAL DEVELOPMENT MODE

**Do NOT use `--evaluate` yet.**

Open Terminal 1:

```bash
cd ~/pacbot_ws/task1b
./task_1b_launch
```

Then open Terminal 2:

```bash
cd ~/pacbot_ws/task1b
python3 task_1b.py
```

Watch the MuJoCo simulator and controller terminal.

The controller should print sensor values similar to:

```text
fl=...
fr=...
sl=...
sr=...
yaw_rate=...
dt=...
```

---

# 4. CONFIRM SENSOR ORIENTATION

The current controller assumes:

```text
fl = front-left
fr = front-right

sl = left-side
sr = right-side
```

However, the original boilerplate comments and the Task 1B sensor description appeared inconsistent.

Therefore, **verify this using the actual MuJoCo simulator before final tuning.**

Observe what happens to each value when:

* PacBot approaches a front wall
* PacBot is close to the left wall
* PacBot is close to the right wall

Report the observed behavior.

---

# 5. TEST THE CURRENT CONTROLLER

Check:

* Does PacBot move forward?
* Does it follow a wall?
* Does it maintain approximately the desired wall distance?
* Does it turn when a wall is detected ahead?
* Does it turn in the correct direction?
* Does it collide with walls?
* Does it lose the wall?
* Does it get stuck spinning?
* Can it eventually exit the maze?

**Do not assume the current PID values are final.**

---

# 6. TUNE THE CONTROLLER

If PacBot is unstable or does not follow the wall correctly, tune:

```text
BASE_SPEED
TARGET_WALL_DISTANCE
KP
KI
KD
MAX_CORRECTION
FRONT_THRESHOLD
```

Also check the wheel-sign convention and turning logic.

The current front-obstacle action is:

```text
left = 0.5
right = -0.5
```

This is only a first test.

If the robot turns incorrectly, the turning logic needs to be corrected.

---

# 7. DEVELOPMENT SUCCESS CRITERIA

Before evaluation, the robot should be tested in normal development mode until it can reliably:

1. Start moving.
2. Follow the selected wall.
3. Correct its position using PID.
4. Detect front obstacles.
5. Turn correctly at corners.
6. Continue through the maze.
7. Exit the maze without manual intervention.

After every code modification:

```bash
python3 -m py_compile task_1b.py
```

---

# 8. ONLY AFTER THE DEVELOPMENT TEST WORKS — RUN EVALUATION

Open Terminal 1:

```bash
cd ~/pacbot_ws/task1b
./task_1b_launch --evaluate
```

Wait until the simulator says:

```text
Evaluation mode enabled
```

Then Terminal 2:

```bash
cd ~/pacbot_ws/task1b
python3 task_1b.py
```

Allow the official evaluation to finish.

**Do not manually create or modify `result.json`.**

The official evaluator must generate it.

---

# 9. CHECK `result.json`

After evaluation:

```bash
cd ~/pacbot_ws/task1b
ls -lh result.json
cat result.json
```

Verify that the file was genuinely generated by the evaluator.

If the robot fails the maze, return to normal development mode and continue tuning.

**Never fabricate a successful result.**

---

# 10. CREATE FINAL SUBMISSION ZIP

The final ZIP should contain only:

```text
result.json
task_1b.py
```

at the top level.

Create the ZIP using the required team ID:

```bash
cd ~/pacbot_ws/task1b
rm -f PB#<TEAM_ID>.zip
zip PB#<TEAM_ID>.zip result.json task_1b.py
```

Then verify:

```bash
unzip -l PB#<TEAM_ID>.zip
```

The listing should contain only:

```text
result.json
task_1b.py
```

Replace `<TEAM_ID>` with the actual team ID.

---

# 11. VIDEO

Record the required Task 1B demonstration/evaluation as **one continuous, unbroken screen recording**.

The recording should clearly show:

* MuJoCo simulator
* Robot movement
* Relevant terminal/controller output
* The actual official run

Do not splice together separate runs or use a fake simulator.

Upload the required video according to the competition instructions.

---

# IMPORTANT RULES

* Do NOT modify `task_1b_launch`.
* Do NOT decompile the encrypted/official simulator.
* Do NOT fabricate sensor data.
* Do NOT fabricate `result.json`.
* Do NOT use a fake simulator to claim completion.
* Do NOT run final evaluation before development testing.
* Keep the final submission ZIP limited to the required files.
* Keep the controller as the team's own work and ensure it complies with the competition's plagiarism rules.

---

# Current Handoff Point

**Work completed by previous developer:**

`task_1b.py` → initial wall-following + PID controller completed and syntax checked.

**Your responsibility now:**

```text
Copy controller
      ↓
Run REAL MuJoCo
      ↓
Verify sensor orientation
      ↓
Observe robot behavior
      ↓
Tune PID + wall following + turning
      ↓
Successfully navigate maze
      ↓
Run official --evaluate
      ↓
Obtain genuine result.json
      ↓
Create final PB#TEAM_ID.zip
      ↓
Record required video
      ↓
Submit
```

**Do not skip the real MuJoCo testing stage.**
