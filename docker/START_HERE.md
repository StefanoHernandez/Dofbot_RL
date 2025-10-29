# 🚀 Inizia Qui - Setup Completo DOFBOT con GPU

## Setup Iniziale (Una volta sola)

### 1. Installa NVIDIA Container Toolkit

**IMPORTANTE**: Questo è necessario per usare la GPU in Docker.

```bash
cd /home/stefano/Documenti/progetti_personali/DOFBOT/docker
sudo ./setup_nvidia_docker.sh
```

Lo script installerà e configurerà automaticamente tutto. Alla fine dovresti vedere un test GPU andato a buon fine.

### 2. Avvia il Container

```bash
./start.sh
```

Prima volta: scaricherà l'immagine base e builderà il container (~5-10 min).

### 3. Entra nel Container

```bash
docker exec -it dofbot_simulation bash
```

### 4. Setup Workspace (prima volta o dopo modifiche)

Dentro il container:

```bash
./setup_workspace.sh
```

Questo builderà tutti i pacchetti ROS.

### 5. Testa GPU

```bash
./test_gpu.sh
```

Dovresti vedere `nvidia-smi` funzionante con la tua RTX A1000.

### 6. Testa Simulazione

```bash
./test_simulation.sh
```

Scegli **opzione 2**: Gazebo + MoveIt + RViz

## Cosa Aspettarsi

Dovresti vedere:

1. **Gazebo** - Simulatore 3D con il robot DOFBOT
   - Braccio a 5 giunti
   - Camera montata
   - Fisica realistica

2. **RViz** - Interfaccia di visualizzazione
   - Modello 3D del robot
   - Pannello MotionPlanning (a sinistra)
   - Marker interattivo (pallina colorata all'end-effector)

3. **Performance GPU**
   - Gazebo dovrebbe girare a 60 FPS (vs 15 FPS su CPU)
   - Rendering fluido
   - No lag

## Test Movimento

In RViz:

1. **Trascina** il marker interattivo (la pallina) su una nuova posizione
2. Clicca **"Plan"** nel pannello MotionPlanning
   - Vedrai una preview della traiettoria
3. Clicca **"Execute"**
   - Il robot si muoverà sia in RViz che in Gazebo!

## Verifica Uso GPU

In un **altro terminale** (sul tuo PC, non nel container):

```bash
watch -n 1 nvidia-smi
```

Mentre Gazebo è in esecuzione, dovresti vedere:
- Processi `gzserver` e `gzclient`
- Utilizzo GPU al 20-40%
- Memoria GPU usata ~300-500 MB

## Struttura File

```
docker/
├── START_HERE.md           ← Questa guida
├── GPU_SETUP.md            ← Dettagli setup GPU
├── QUICKSTART.md           ← Guida rapida comandi
├── README.md               ← Documentazione completa
│
├── setup_nvidia_docker.sh  ← Installa NVIDIA toolkit (run una volta)
├── start.sh                ← Avvia container
├── check_prerequisites.sh  ← Verifica prerequisiti
│
├── setup_workspace.sh      ← Build workspace ROS (dentro container)
├── test_simulation.sh      ← Menu test simulazione (dentro container)
├── test_gpu.sh             ← Test GPU (dentro container)
│
├── Dockerfile              ← Definizione immagine ROS Melodic
└── docker-compose.yml      ← Configurazione container con GPU
```

## Workflow Tipico

```bash
# 1. Sul tuo PC (una volta al giorno)
cd /home/stefano/Documenti/progetti_personali/DOFBOT/docker
./start.sh

# 2. Entra nel container
docker exec -it dofbot_simulation bash

# 3. Dentro container - prima volta setup
./setup_workspace.sh

# 4. Dentro container - ogni volta che vuoi testare
./test_simulation.sh
# Scegli opzione 2

# 5. In RViz - testa movimento
# Drag & drop marker → Plan → Execute

# 6. Quando finisci - esci dal container
exit

# 7. Sul tuo PC - ferma container
docker compose down
```

## Comandi Docker Utili

```bash
# Vedere container attivi
docker ps

# Vedere log container
docker logs dofbot_simulation

# Fermare container
docker compose down

# Riavviare container esistente
docker compose start

# Ricostruire dopo modifiche Dockerfile
docker compose build --no-cache

# Pulire tutto e ripartire
docker compose down
docker rmi dofbot_melodic:latest
./start.sh
```

## Troubleshooting Veloce

### GPU non funziona
```bash
# Sul PC
sudo ./setup_nvidia_docker.sh

# Poi riavvia container
docker compose down
./start.sh
```

### Gazebo/RViz non si apre
```bash
# Sul PC
xhost +local:docker

# Verifica DISPLAY
echo $DISPLAY  # deve essere tipo :0 o :1
```

### Build workspace fallisce
```bash
# Dentro container
cd /root/dofbot_ws
rm -rf build devel
./setup_workspace.sh
```

### Container non parte
```bash
# Verifica Docker daemon
sudo systemctl status docker

# Riavvia se necessario
sudo systemctl restart docker
```

## Prossimi Passi

Dopo aver verificato che tutto funziona:

### Fase 1: Test Funzionalità Esistenti ✅
- [x] Gazebo simulation
- [x] MoveIt motion planning
- [x] RViz visualization
- [ ] Test script motion planning Python
- [ ] Test servizio kinematics (IK/FK)
- [ ] Test YOLOv5 object detection

### Fase 2: Valutazione per RL 🔄
- [ ] Decidere: continuare con Gazebo o passare a MuJoCo?
- [ ] Valutare integrazione YOLO11 vs YOLOv5
- [ ] Definire reward function per pick-and-place
- [ ] Definire action space e observation space
- [ ] Scegliere algoritmo RL (PPO vs SAC)

### Fase 3: Implementazione RL 📝
- [ ] Setup PyTorch + TorchRL nel container
- [ ] Creare environment Gym per DOFBOT
- [ ] Implementare reward shaping
- [ ] Aggiungere object spawning randomizzato
- [ ] Training loop base
- [ ] Logging con TensorBoard
- [ ] Sim2Real transfer preparation

## Note Importanti

- ✅ Il container monta i file dal tuo PC: modifiche ai sorgenti sono persistenti
- ✅ GPU configurata per Gazebo, PyTorch, e YOLO
- ✅ ROS Melodic (Python 2.7) - per Python 3 useremo virtualenv
- ✅ Network mode `host` - ROS comunica senza problemi
- ⚠️ YOLOv5 attuale richiede Python 3 - da configurare separatamente
- ⚠️ Per training RL serviranno librerie aggiuntive (TorchRL, Gym, ecc.)

## Documentazione

- [GPU_SETUP.md](GPU_SETUP.md) - Setup dettagliato GPU e troubleshooting
- [QUICKSTART.md](QUICKSTART.md) - Riferimento rapido comandi
- [README.md](README.md) - Documentazione tecnica completa

## Supporto

Se incontri problemi:

1. Controlla la sezione Troubleshooting in questo file
2. Leggi [GPU_SETUP.md](GPU_SETUP.md) per problemi GPU
3. Controlla log: `docker logs dofbot_simulation`
4. Verifica prerequisiti: `./check_prerequisites.sh`

---

**Pronto?** Inizia con:
```bash
sudo ./setup_nvidia_docker.sh
```
