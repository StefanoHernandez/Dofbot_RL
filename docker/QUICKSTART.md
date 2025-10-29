# DOFBOT Quick Start Guide

## 🚀 Primo avvio (dal terminale host)

```bash
cd docker
./start.sh
```

Questo configura X11 e avvia il container.

## 🐳 Entrare nel container

```bash
docker exec -it dofbot_simulation bash
```

## ⚙️ Setup workspace (solo la prima volta o dopo modifiche)

Dentro il container:

```bash
./setup_workspace.sh
```

## 🎮 Testare la simulazione

Dentro il container:

```bash
./test_simulation.sh
```

Scegli **opzione 2** per il test completo: Gazebo + MoveIt + RViz

## 📝 Cosa aspettarsi

Dovresti vedere:

1. **Gazebo** - simulazione fisica del robot DOFBOT
2. **RViz** - visualizzazione con pannello MotionPlanning

### Test movimento:

In RViz:
1. Trascina il marker interattivo (pallina colorata) su una nuova posizione
2. Clicca "Plan" nel pannello MotionPlanning
3. Clicca "Execute" per eseguire il movimento

Il robot dovrebbe muoversi sia in RViz che in Gazebo!

## 🛑 Fermare l'ambiente

Sul terminale host (fuori dal container):

```bash
cd docker
docker compose down  # or docker-compose down
```

## 🔧 Comandi utili

### Riavviare container esistente
```bash
docker compose start  # or docker-compose start
```

### Vedere log del container
```bash
docker logs dofbot_simulation
```

### Ricostruire immagine (dopo modifiche al Dockerfile)
```bash
docker compose build --no-cache  # or docker-compose build --no-cache
```

### Pulire tutto e ripartire
```bash
docker compose down  # or docker-compose down
docker rmi dofbot_melodic:latest
./start.sh
```

## 📂 Struttura file

```
docker/
├── start.sh              ← Esegui questo per primo (host)
├── setup_workspace.sh    ← Esegui dentro container
├── test_simulation.sh    ← Menu interattivo test (container)
├── Dockerfile            ← Definizione ambiente ROS Melodic
├── docker-compose.yml    ← Configurazione container
├── README.md             ← Documentazione completa
└── QUICKSTART.md         ← Questa guida rapida
```

## ⚡ Workflow completo

```bash
# 1. Sul tuo PC (host)
cd /home/stefano/Documenti/progetti_personali/DOFBOT/docker
./start.sh

# 2. Entra nel container
docker exec -it dofbot_simulation bash

# 3. Setup workspace (prima volta)
./setup_workspace.sh

# 4. Testa simulazione
./test_simulation.sh
# Scegli opzione 2

# 5. In RViz: trascina marker → Plan → Execute

# 6. Quando hai finito, esci dal container (Ctrl+D) e fermalo:
# (di nuovo sul terminale host)
docker-compose down
```

## 🐛 Problemi comuni

### GUI non si apre
```bash
# Sul terminale host
xhost +local:docker
```

### Errori di build workspace
```bash
# Dentro container
cd /root/dofbot_ws
rm -rf build devel
./setup_workspace.sh
```

### Container non parte
```bash
# Verifica Docker
docker ps -a
docker logs dofbot_simulation

# Riavvia
docker-compose restart
```

## 📚 Prossimi passi

Dopo aver verificato che la simulazione funziona:

1. ✅ Testare script Python esistenti per motion planning
2. ✅ Verificare servizio kinematics (IK/FK)
3. ✅ Valutare cosa serve per il progetto RL:
   - Aggiungere spawning oggetti
   - Integrare YOLO11
   - Setup training loop con TorchRL
   - Decidere se continuare con Gazebo o passare a MuJoCo

## 💡 Note

- I file sorgente sono montati dal tuo PC, le modifiche sono persistenti
- Il workspace si ricostruisce ogni volta che lanci `setup_workspace.sh`
- ROS Melodic usa Python 2.7 (YOLOv5 richiederà Python 3, da gestire)
- Network mode è `host` per comunicazione ROS
