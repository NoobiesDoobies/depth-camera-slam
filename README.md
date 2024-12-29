# How to run

## Step 1: Clone directory
```
mkdir -p ~/slam_bot/src
cd ~/slam_bot/src 
git clone git@github.com:NoobiesDoobies/depth-camera-slam.git
```

## Step 2: Build
```
cd ~/slam_bot 
colcon build --symlink-install
```

## Step 3: Launch simulation
```
ros2 launch articubot_one launch_sim.launch.py
```

## Step 4: Launch SLAM
```
ros2 launch articubot_one slam.launch.py
```

