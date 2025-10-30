# DOFBOT ROS Noetic + MuJoCo Setup

## Overview

Questo setup utilizza ROS Noetic (Ubuntu 20.04) con supporto completo per:
- **ROS 1 Noetic** - Ultima versione di ROS 1 con supporto LTS
- **Python 3.8** - Necessario per MuJoCo e librerie RL moderne
- **MuJoCo 3.0+** - Simulatore fisico veloce per Reinforcement Learning
- **Gymnasium** - Framework standard per ambienti RL
- **Stable-Baselines3** - Algoritmi RL pronti all'uso
- **GPU NVIDIA** - Supporto completo per rendering e training

## Differenze vs ROS Melodic

| Caratteristica | Melodic (vecchio) | Noetic (nuovo) |
|----------------|-------------------|----------------|
| Ubuntu | 18.04 | 20.04 |
| Python | 2.7 | 3.8 |
| MuJoCo | ❌ Non supportato | ✅ Supportato |
| RL Libraries | ❌ Limitate | ✅ Complete |
| Compatibilità | ✅ ROS 1 | ✅ ROS 1 |

## Quick Start

### 1. Build e avvio container

```bash
cd docker/

# Build immagine (prima volta, ~5-10 minuti)
docker compose -f docker-compose.noetic.yml build

# Avvia container
docker compose -f docker-compose.noetic.yml up -d

# Entra nel container
docker exec -it dofbot_noetic_simulation bash
```

### 2. Build workspace ROS

```bash
# Nel container:
cd /root/dofbot_ws
catkin_make

# Source environment
source /opt/ros/noetic/setup.bash
source devel/setup.bash

# Verifica pacchetti
rospack list | grep dofbot
```

### 3. Test MuJoCo

```bash
# Nel container:
python3 -c "import mujoco; print('MuJoCo version:', mujoco.__version__)"
python3 -c "import gymnasium; print('Gymnasium OK')"
```

### 4. Lancia RViz + MoveIt

```bash
# Nel container:
roslaunch dofbot_moveit demo.launch
```

## Struttura Directory

```
docker/
├── Dockerfile.noetic          # Nuovo Dockerfile con ROS Noetic
├── docker-compose.noetic.yml  # Compose file per Noetic
├── mujoco_models/             # Modelli MuJoCo (MJCF/URDF)
└── rl_scripts/                # Script Python per RL
```

## Librerie Python Installate

### Core
- `mujoco>=3.0.0` - Simulatore fisico
- `gymnasium>=0.29.0` - Framework RL
- `stable-baselines3>=2.0.0` - Algoritmi RL (PPO, SAC, TD3, etc.)
- `numpy>=1.21.0`
- `opencv-python>=4.5.0`

### ML/DL
- `torch` - PyTorch per neural networks
- `tensorboard` - Logging e visualizzazione

### Utilities
- `matplotlib`, `seaborn` - Plotting
- `pandas` - Data analysis
- `jupyter` - Notebook interattivi
- `tqdm` - Progress bars

## Workflow Reinforcement Learning

### 1. Converti URDF → MJCF (MuJoCo format)

```python
import mujoco

# MuJoCo può caricare URDF direttamente
model = mujoco.MjModel.from_xml_path('/root/dofbot_ws/src/dofbot_moveit/urdf/dofbot.urdf')
```

### 2. Crea ambiente Gymnasium

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np

class DofbotEnv(gym.Env):
    def __init__(self):
        super().__init__()
        self.model = mujoco.MjModel.from_xml_path('dofbot.urdf')
        self.data = mujoco.MjData(self.model)

        # Define action space (joint positions/velocities)
        self.action_space = spaces.Box(
            low=-1, high=1, shape=(5,), dtype=np.float32
        )

        # Define observation space
        self.observation_space = spaces.Box(
            low=-np.inf, high=np.inf, shape=(15,), dtype=np.float32
        )

    def reset(self, seed=None, options=None):
        super().reset(seed=seed)
        mujoco.mj_resetData(self.model, self.data)
        return self._get_obs(), {}

    def step(self, action):
        # Apply action
        self.data.ctrl[:] = action
        mujoco.mj_step(self.model, self.data)

        obs = self._get_obs()
        reward = self._compute_reward()
        terminated = self._check_termination()

        return obs, reward, terminated, False, {}
```

### 3. Training con Stable-Baselines3

```python
from stable_baselines3 import PPO
from stable_baselines3.common.env_checker import check_env

# Check environment
env = DofbotEnv()
check_env(env)

# Train
model = PPO("MlpPolicy", env, verbose=1)
model.learn(total_timesteps=100000)

# Save
model.save("dofbot_ppo")

# Test
obs, _ = env.reset()
for i in range(1000):
    action, _states = model.predict(obs)
    obs, reward, done, truncated, info = env.step(action)
    if done:
        obs, _ = env.reset()
```

## GPU Support

Il container supporta GPU NVIDIA per:
- **MuJoCo rendering** - Visualizzazione accelerata
- **PyTorch training** - Training neural networks
- **Gazebo** - Simulazione fisica

Verifica GPU:
```bash
nvidia-smi
python3 -c "import torch; print('CUDA available:', torch.cuda.is_available())"
```

## Troubleshooting

### Build fallisce
```bash
# Clean e rebuild
docker compose -f docker-compose.noetic.yml down
docker compose -f docker-compose.noetic.yml build --no-cache
```

### MuJoCo non trova OpenGL
```bash
# Nel container:
export MUJOCO_GL=egl  # O 'osmesa' per CPU-only
```

### Workspace non compila
```bash
# Nel container:
cd /root/dofbot_ws
rm -rf build/ devel/
rosdep install --from-paths src --ignore-src -r -y
catkin_make
```

## Next Steps

1. ✅ Build container e workspace
2. ✅ Verifica MuJoCo installazione
3. 🔄 Converti URDF DOFBOT per MuJoCo
4. 🔄 Crea ambiente Gymnasium custom
5. 🔄 Train RL policy (PPO/SAC)
6. 🔄 Deploy policy su robot reale

## References

- [MuJoCo Documentation](https://mujoco.readthedocs.io/)
- [Gymnasium Documentation](https://gymnasium.farama.org/)
- [Stable-Baselines3 Documentation](https://stable-baselines3.readthedocs.io/)
- [ROS Noetic Documentation](http://wiki.ros.org/noetic)
