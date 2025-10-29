# NVIDIA GPU Setup per Docker CLI Daemon

Questa guida ti aiuta a configurare il supporto GPU per Docker usando il daemon CLI invece di Docker Desktop.

## Perché usare Docker CLI daemon?

- ✅ Supporto GPU completo per Gazebo e training RL
- ✅ Migliori performance (no overhead di Docker Desktop)
- ✅ Accesso diretto all'hardware NVIDIA
- ✅ Necessario per CUDA/PyTorch con GPU

## Prerequisiti

1. **NVIDIA Driver installato** (driver >= 470.x)
   ```bash
   nvidia-smi
   ```
   Dovresti vedere la tua GPU (nel tuo caso: NVIDIA RTX A1000 6GB)

2. **Docker installato** (versione >= 20.10)
   ```bash
   docker --version
   ```

3. **Non stai usando Docker Desktop** (o è stato fermato)

## Installazione NVIDIA Container Toolkit

### Metodo Automatico (Consigliato)

Esegui lo script fornito:

```bash
sudo ./setup_nvidia_docker.sh
```

Questo script:
1. Verifica che il driver NVIDIA sia installato
2. Installa NVIDIA Container Toolkit
3. Configura Docker daemon per usare il runtime NVIDIA
4. Riavvia Docker daemon
5. Testa il supporto GPU

### Metodo Manuale

Se preferisci installare manualmente:

```bash
# 1. Aggiungi repository NVIDIA
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | \
  sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
  sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
  sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

# 2. Installa toolkit
sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit

# 3. Configura Docker
sudo nvidia-ctk runtime configure --runtime=docker

# 4. Riavvia Docker
sudo systemctl restart docker

# 5. Testa
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

## Verifica Installazione

Dopo l'installazione, verifica che funzioni:

```bash
docker run --rm --gpus all nvidia/cuda:11.8.0-base-ubuntu22.04 nvidia-smi
```

Dovresti vedere l'output di `nvidia-smi` che mostra la tua GPU.

## Configurazione DOFBOT Container

Il file `docker-compose.yml` è già configurato per usare la GPU:

```yaml
environment:
  - NVIDIA_VISIBLE_DEVICES=all
  - NVIDIA_DRIVER_CAPABILITIES=all

deploy:
  resources:
    reservations:
      devices:
        - driver: nvidia
          count: all
          capabilities: [gpu, compute, utility]
```

## Avvio Container con GPU

Usa lo script normale:

```bash
./start.sh
```

Il container verrà avviato automaticamente con supporto GPU.

## Test GPU nel Container

Una volta dentro il container:

```bash
docker exec -it dofbot_simulation bash

# Test GPU
./test_gpu.sh
```

Output atteso:
```
========================================
GPU Test inside DOFBOT Container
========================================

Test 1: nvidia-smi availability
----------------------------------------------
+-----------------------------------------------------------------------------+
| NVIDIA-SMI 580.95.05    Driver Version: 580.95.05    CUDA Version: 12.8     |
|-------------------------------+----------------------+----------------------+
| GPU  Name        Persistence-M| Bus-Id        Disp.A | Volatile Uncorr. ECC |
| Fan  Temp  Perf  Pwr:Usage/Cap|         Memory-Usage | GPU-Util  Compute M. |
|===============================+======================+======================|
|   0  NVIDIA RTX A100...  Off  | 00000000:01:00.0 Off |                  N/A |
| N/A   45C    P8     5W /  N/A |    256MiB /  6144MiB |      0%      Default |
+-------------------------------+----------------------+----------------------+

✓ nvidia-smi works!
```

## Verifica Utilizzo GPU durante Simulazione

### Metodo 1: Monitor in tempo reale

In un terminale separato (fuori dal container):

```bash
watch -n 1 nvidia-smi
```

### Metodo 2: Durante simulazione Gazebo

1. Avvia simulazione:
   ```bash
   # Dentro container
   roslaunch dofbot_config demo_gazebo.launch
   ```

2. In altro terminale, controlla processi GPU:
   ```bash
   nvidia-smi
   ```

Dovresti vedere processi `gzserver` o `gzclient` che usano la GPU.

### Output Atteso

```
+-----------------------------------------------------------------------------+
| Processes:                                                                  |
|  GPU   GI   CI        PID   Type   Process name                  GPU Memory |
|        ID   ID                                                   Usage      |
|=============================================================================|
|    0   N/A  N/A    123456      C   gzserver                          250MiB |
|    0   N/A  N/A    123457      G   gzclient                          150MiB |
+-----------------------------------------------------------------------------+
```

## Benefici per il Progetto DOFBOT

Con GPU abilitata ottieni:

1. **Gazebo più veloce**
   - Rendering accelerato
   - Simulazione fisica più rapida
   - Più FPS in RViz

2. **Training RL accelerato**
   - PyTorch usa GPU automaticamente
   - Training 10-100x più veloce
   - Batch più grandi possibili

3. **Inferenza YOLO più veloce**
   - YOLO11 su GPU: ~100-200 FPS
   - YOLO11 su CPU: ~5-20 FPS
   - Real-time object detection garantito

4. **Simulazioni multiple parallele**
   - Puoi eseguire N ambienti in parallelo
   - Utile per algoritmi come PPO/SAC
   - Massimizza uso GPU

## Troubleshooting

### GPU non rilevata nel container

```bash
# Verifica variabili d'ambiente
echo $NVIDIA_VISIBLE_DEVICES  # dovrebbe essere "all"
echo $NVIDIA_DRIVER_CAPABILITIES  # dovrebbe essere "all"

# Verifica device nodes
ls -la /dev/nvidia*
```

### Docker compose non usa GPU

Se usi `docker-compose` (V1):
```bash
# V1 non supporta bene deploy.resources
# Usa invece docker compose (V2)
docker compose up -d
```

### Driver version mismatch

```bash
# Aggiorna driver NVIDIA
sudo ubuntu-drivers autoinstall
sudo reboot

# Reinstalla container toolkit
sudo apt-get install --reinstall nvidia-container-toolkit
sudo systemctl restart docker
```

### Gazebo non usa GPU

Verifica che Gazebo sia compilato con supporto GPU:
```bash
# Dentro container
gzserver --version
# Cerca "OGRE" o "GPU" nell'output
```

Se necessario, abilita rendering GPU in Gazebo:
```xml
<!-- In .gazebo/gui.ini -->
[rendering]
gpu_laser_count = 4
```

## Comandi Utili

```bash
# Monitoraggio GPU continuo
watch -n 0.5 nvidia-smi

# Info dettagliate GPU
nvidia-smi -q

# Temperatura e consumo
nvidia-smi --query-gpu=temperature.gpu,power.draw --format=csv,noheader --loop=1

# Processi che usano GPU
nvidia-smi pmon

# Test CUDA nel container
docker exec -it dofbot_simulation nvidia-smi

# Verifica PyTorch vede GPU (quando installato)
docker exec -it dofbot_simulation python3 -c "import torch; print(torch.cuda.is_available())"
```

## Performance Attese

Con RTX A1000 (6GB):

| Task | CPU | GPU | Speedup |
|------|-----|-----|---------|
| Gazebo rendering | 15 FPS | 60 FPS | 4x |
| YOLO11 inference | 10 FPS | 150 FPS | 15x |
| PPO training (1M steps) | 2 ore | 10 min | 12x |
| SAC training (1M steps) | 3 ore | 15 min | 12x |

## Prossimi Passi

Dopo aver configurato la GPU:

1. ✅ Testa Gazebo con `demo_gazebo.launch`
2. ✅ Verifica uso GPU con `nvidia-smi`
3. ✅ Installa PyTorch con CUDA support
4. ✅ Testa YOLO11 su GPU
5. ✅ Setup TorchRL per training RL

## Riferimenti

- [NVIDIA Container Toolkit](https://docs.nvidia.com/datacenter/cloud-native/container-toolkit/install-guide.html)
- [Docker GPU support](https://docs.docker.com/config/containers/resource_constraints/#gpu)
- [Gazebo GPU acceleration](http://gazebosim.org/tutorials?tut=guided_i1)
