# Quick Start: Testing the CV Predictor with Hover Demo

## What You Need Running

To test the predictor, you need **3 terminals** with these commands:

### Terminal 1: PX4 SITL (✓ Already Running)
```bash
cd ~/PX4-Autopilot
make px4_sitl gz_x500
```
**Status**: You already have this running!

### Terminal 2: MicroXRCEAgent (✓ Already Running)
```bash
MicroXRCEAgent udp4 -p 8888
```
**Status**: You already have this running!

### Terminal 3: Hover Demo (← YOU NEED TO START THIS)
```bash
cd ~/Documents/Orion/orion_arm
source install/setup.bash
./hover_demo.sh
```

This will:
- Make the drone take off and fly in a straight line
- Show RViz with the drone visualization
- Provide the target data for the predictor to track

### Terminal 4: Predictor Demo (← THEN RUN THIS)
```bash
cd ~/Documents/Orion/orion_arm
./hover_predictor_demo.sh
```

This will:
- Ask you to select CV or CA predictor
- Start the predictor node
- Add prediction visualizations to RViz

## Step-by-Step Instructions

1. **Check your current terminals:**
   - Terminal 1: PX4 SITL running ✓
   - Terminal 2: MicroXRCEAgent running ✓

2. **Open a NEW terminal and start the hover demo:**
   ```bash
   cd ~/Documents/Orion/orion_arm
   source install/setup.bash
   ./hover_demo.sh
   ```
   
   - Follow the prompts
   - Answer 'y' when asked if PX4 is running
   - Say 'n' to MicroXRCEAgent (already running in Terminal 2)
   - Wait for RViz to open and show the drone

3. **Once the drone is flying, open ANOTHER new terminal:**
   ```bash
   cd ~/Documents/Orion/orion_arm
   ./hover_predictor_demo.sh
   ```
   
   - Answer 'y' when asked if PX4 is running
   - Select '1' for CV predictor (or '2' for CA)
   - Answer 'y' when asked if hover demo is running
   - Say 'n' to MicroXRCEAgent (already running)

4. **Watch RViz:**
   - You should see GREEN/YELLOW/RED markers appearing
   - These are the predicted future positions
   - Ellipsoids show uncertainty (growing with time)
   - The predicted trajectory should match the drone's actual path

## What You Should See

In RViz, you'll see:
- **Red sphere**: Current drone position
- **Red line**: Past trajectory trail
- **Green/Yellow/Red spheres**: Predicted positions at t+0.5s, t+1s, t+2s, t+3s, t+5s
- **Ellipsoids**: Uncertainty regions (bigger = less certain)
- **Green line**: Predicted future trajectory

For straight-line motion, the CV predictor should be very accurate!

## Troubleshooting

**"No predictions visible"**
- Make sure both hover demo AND predictor demo are running
- Check: `ros2 topic list | grep prediction`
- Should see `/target/predicted_state` and `/target/prediction_markers`

**"hover_demo.sh not found"**
- You might not have the hover demo script
- Use manual command: `ros2 launch attack_drone hover.launch.py`

**"Predictor not starting"**
- Check that drone is publishing data:
  ```bash
  ros2 topic hz /px4_1/fmu/out/vehicle_local_position
  ```
  Should show ~50 Hz

## Alternative: Manual Launch

If the scripts don't work, launch manually:

```bash
# Terminal 3: Hover demo
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch attack_drone hover.launch.py

# Terminal 4: CV Predictor
cd ~/Documents/Orion/orion_arm
source install/setup.bash
ros2 launch orion_flight hover_with_predictor.launch.py predictor_type:=cv
```

## Summary

**Terminal Setup:**
1. PX4 SITL (running) ✓
2. MicroXRCEAgent (running) ✓  
3. Hover Demo (← start this first)
4. Predictor Demo (← then start this)

Both demos will share the same RViz window and you'll see predictions overlaid on the actual drone trajectory!
