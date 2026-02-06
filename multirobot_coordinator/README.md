# multirobot_coordinator

Package ROS2 Python (ament_python) per la coordinazione di robot multipli con detection ArUco e navigazione autonoma.

---

## STRUTTURA DEL PACKAGE

```
multirobot_coordinator/
│
├── multirobot_coordinator/          # ← CODICE PYTHON (modulo Python)
│   ├── __init__.py                  # Marca la cartella come package Python
│   ├── task_allocator_node.py       # Nodo: ArUco ID → Settore
│   ├── fra2mo_executor_node.py      # Nodo: Navigazione robot mobile
│   └── iiwa_scanner_node.py         # Nodo: Controllo braccio IIWA
│
├── launch/                          # Launch files ROS2
│   └── bringup.launch.py            # Avvia i 3 nodi + ArUco detection
│
├── config/                          # File di configurazione YAML
│   ├── aruco_config.yaml            # Parametri detection ArUco
│   └── scanner_config.yaml          # Posizioni joint braccio IIWA
│
├── gazebo/                          # Modelli Gazebo
│   └── models/                      # Marker ArUco 3D (4 settori)
│       ├── aruco_tag_5/             # Marker Nord
│       ├── aruco_tag_10/            # Marker Est
│       ├── aruco_tag_15/            # Marker Sud
│       └── aruco_tag_20/            # Marker Ovest
│
├── resource/                        # ROS2 package indexing
│   └── multirobot_coordinator       # File marker vuoto (per ros2 pkg list)
│
├── package.xml                      # Metadati e dipendenze ROS2
├── setup.py                         # Build configuration (equivalente CMakeLists.txt)
├── setup.cfg                        # Configurazione installazione script
└── README.md                        # ← Questo file
```

---

## PERCHÉ NON C'È `src/`?

Questo è un **package Python** (ament_python), non C++ (ament_cmake).

### Differenza tra C++ e Python:

| C++ (ament_cmake) | Python (ament_python) |
|-------------------|----------------------|
| `CMakeLists.txt` | `setup.py` |
| `src/` + `include/` | `package_name/` |
| Compilato (g++) | Interpretato |
| Header files (.h) | Nessun header |

**Struttura Python ROS2:**
- Il codice va nella **cartella con nome identico al package**
- Questa cartella diventa un **modulo Python importabile**
- Non serve separare header/source come in C++

---

## FILE PRINCIPALI

### **1. setup.py** - Build Configuration
File Python che definisce:
- Come installare il package
- Dove copiare launch files, config, modelli
- Quali eseguibili creare (`task_allocator_node`, ecc.)

Equivalente di `CMakeLists.txt` per package C++.

### **2. package.xml** - Metadati ROS2
Contiene:
- Nome e versione del package
- Dipendenze (`rclpy`, `aruco_msgs`, `nav2_msgs`, ecc.)
- Tipo di build (`ament_python`)

### **3. setup.cfg** - Installazione Script
Dice a Python dove installare gli eseguibili:
```
install/multirobot_coordinator/lib/multirobot_coordinator/
```

### **4. resource/multirobot_coordinator**
File **vuoto** (0 bytes) che serve solo per:
- ROS2 package indexing
- `ros2 pkg list` trova il package
- `get_package_share_directory()` funziona

---

## NODI ROS2

### **1. task_allocator_node.py**
**Ruolo:** Task allocation basata su ArUco detection

**Workflow:**
```
Riceve MarkerArray (/aruco_marker_publisher/markers)
    ↓
Mappa ID → Settore (5→N, 10→E, 15→S, 20→W)
    ↓
Pubblica String su /task/sector
```

**Design:** Stateless, usa mappatura statica invece di geometria

---

### **2. fra2mo_executor_node.py**
**Ruolo:** Executor navigazione robot mobile

**Workflow:**
```
Riceve Settore (/task/sector)
    ↓
Converte Settore → Coordinate goal
    ↓
Invia NavigateToPose action a Nav2
    ↓
Monitora stato (busy/idle)
```

**Design:** Gestisce stato del robot, ignora comandi se busy

---

### **3. iiwa_scanner_node.py**
**Ruolo:** Controllo braccio robotico IIWA

**Workflow:**
```
Riceve Settore (/task/sector)
    ↓
Legge posizione joint da scanner_config.yaml
    ↓
Muove 6 joint del braccio
    ↓
Orienta camera verso il marker
```

**Design:** Posizioni hardcoded, movimento simultaneo con robot mobile

---

## CONFIGURAZIONE

### **aruco_config.yaml**
```yaml
marker_size: 0.15        # Dimensione marker in metri
dictionary: 4            # DICT_4X4_50
```

### **scanner_config.yaml**
```yaml
sector_positions:
  N: [0.0, 0.5, 0.0, -1.0, 0.0, 0.0]   # Joint positions per settore Nord
  E: [1.57, 0.3, 0.0, -1.0, 0.0, 0.0]  # Est
  S: [3.14, 0.5, 0.0, -1.0, 0.0, 0.0]  # Sud
  W: [-1.57, 0.3, 0.0, -1.0, 0.0, 0.0] # Ovest
```

---

## BUILD E INSTALLAZIONE

### **Build del package:**
```bash
cd /path/to/workspace
colcon build --packages-select multirobot_coordinator
source install/setup.bash
```

### **Cosa succede durante il build:**

1. **Legge `package.xml`** → Trova dipendenze
2. **Esegue `setup.py`**:
   - Copia launch files in `install/share/multirobot_coordinator/launch/`
   - Copia config YAML in `install/share/.../config/`
   - Copia modelli Gazebo in `install/share/.../gazebo/`
   - Crea eseguibili in `install/lib/multirobot_coordinator/`
3. **Registra package** → `ros2 pkg list` lo trova

### **Risultato:**
```
install/multirobot_coordinator/
├── lib/multirobot_coordinator/
│   ├── task_allocator_node       ← Eseguibile
│   ├── fra2mo_executor_node
│   └── iiwa_scanner_node
└── share/multirobot_coordinator/
    ├── launch/
    ├── config/
    └── gazebo/
```

---

## ESECUZIONE

### **Launch completo:**
```bash
ros2 launch multirobot_coordinator bringup.launch.py
```

Avvia:
- `task_allocator_node` (mapping ArUco)
- `fra2mo_executor_node` (navigazione)
- `iiwa_scanner_node` (braccio)
- `marker_publisher` (detection ArUco)

### **Nodi singoli:**
```bash
ros2 run multirobot_coordinator task_allocator_node
ros2 run multirobot_coordinator fra2mo_executor_node
ros2 run multirobot_coordinator iiwa_scanner_node
```

---

## DIPENDENZE

### **ROS2 Packages:**
- `rclpy` - Python client library
- `aruco_msgs` - Messaggi ArUco (MarkerArray)
- `nav2_msgs` - Messaggi navigazione (NavigateToPose action)
- `sensor_msgs` - JointState
- `trajectory_msgs` - JointTrajectory
- `geometry_msgs` - PoseStamped, Twist
- `std_msgs` - String

### **Packages esterni:**
- `aruco_ros` - Detection marker ArUco

---

## TOPICS

### **Subscriptions:**
- `/aruco_marker_publisher/markers` (MarkerArray) → task_allocator
- `/task/sector` (String) → fra2mo_executor, iiwa_scanner

### **Publications:**
- `/task/sector` (String) → task_allocator
- `/iiwa_arm_controller/joint_trajectory` (JointTrajectory) → iiwa_scanner

### **Actions:**
- `/navigate_to_pose` (NavigateToPose) → fra2mo_executor (client)

---

## TESTING

### **Verifica nodi attivi:**
```bash
ros2 node list
# Output:
# /task_allocator_node
# /fra2mo_executor_node
# /iiwa_scanner_node
# /aruco_marker_publisher
```

### **Monitora detection:**
```bash
ros2 topic echo /aruco_marker_publisher/markers
```

### **Monitora comandi settore:**
```bash
ros2 topic echo /task/sector
```

### **Test manuale:**
```bash
# Pubblica comando settore manualmente
ros2 topic pub /task/sector std_msgs/msg/String "{data: 'N'}" --once
```

---

## WORKFLOW COMPLETO

```
1. Camera vede marker ArUco
    ↓
2. aruco_ros rileva marker, pubblica MarkerArray
    ↓
3. task_allocator riceve MarkerArray
    ↓
4. task_allocator mappa ID → Settore (5→'N')
    ↓
5. task_allocator pubblica String('N') su /task/sector
    ↓
6. fra2mo_executor riceve 'N'
    ↓
7. fra2mo_executor converte 'N' → goal (3.0, 3.0)
    ↓
8. fra2mo_executor invia NavigateToPose action
    ↓
9. iiwa_scanner riceve 'N'
    ↓
10. iiwa_scanner muove braccio verso posizione Nord
    ↓
11. Robot mobile naviga + braccio si orienta
```

---

## DESIGN CHOICES

### **1. Static ID Mapping**
**Scelta:** Usare dizionario ID→Settore invece di trasformazioni geometriche

**Vantaggio:**
- Semplice e robusto
- Non richiede TF complesse (camera_frame → map_frame)
- Immune a errori di stima della posa

**Alternativa:** Usare `marker.pose.pose.position` e trasformare a map frame

---

### **2. Responsabilità Separate**
**task_allocator:**
- Solo rilevamento e mapping (stateless)
- Pubblica sempre immediatamente

**fra2mo_executor:**
- Gestisce stato del robot (busy/idle)
- Debounce e controllo duplicati

**Vantaggio:** Single Responsibility Principle, facile da testare

---

### **3. Python invece di C++**
**Scelta:** Package Python per logica coordinator

**Motivazione:**
- Prototipazione rapida
- Più facile da debuggare
- Non richiede prestazioni real-time critiche
- Nav2 e controller sono già in C++ (le parti performance-critical)

---

## TROUBLESHOOTING

### **Package non trovato:**
```bash
# Rebuild + source
colcon build --packages-select multirobot_coordinator
source install/setup.bash
```

### **Eseguibili non trovati:**
Verifica `setup.py` entry_points e rebuild.

### **Launch file non trovato:**
```bash
ros2 pkg prefix multirobot_coordinator
# Verifica che esista: .../share/multirobot_coordinator/launch/
```

### **Config non caricati:**
```bash
# Verifica percorso in bringup.launch.py
pkg_dir = get_package_share_directory('multirobot_coordinator')
config_file = os.path.join(pkg_dir, 'config', 'aruco_config.yaml')
```

---

## AUTORI

- Francesco
- Progetto: Multi-robot coordination con ArUco markers

## LICENZA

Apache-2.0

---

## NOTE

- Questo è un package **Python puro** (ament_python)
- Non contiene codice C++
- I modelli Gazebo sono inclusi per deployment facile
- Il package companion `multirobot_control` gestisce spawn e controller
