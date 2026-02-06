# ===== SETUP.PY - BUILD CONFIGURATION PER PACKAGE ROS2 PYTHON =====
# Questo file è l'equivalente di CMakeLists.txt per package C++.
# Definisce come installare il package e creare gli eseguibili ROS2.

from setuptools import find_packages, setup  # Libreria standard Python per installazione package
import os  # Per manipolare path dei file
from glob import glob  # Per trovare file con pattern (es. *.py, *.yaml)

# ===== NOME DEL PACKAGE =====
package_name = 'multirobot_coordinator'
# IMPORTANTE: Deve corrispondere a:
# - Nome della cartella che contiene i file Python (multirobot_coordinator/)
# - Nome in package.xml (<name>multirobot_coordinator</name>)

# ===== CONFIGURAZIONE PRINCIPALE =====
setup(
    # Nome del package ROS2
    name=package_name,
    
    # Versione (semantic versioning: MAJOR.MINOR.PATCH)
    version='0.1.0',
    
    # Trova automaticamente tutti i moduli Python nel package
    # Cerca cartelle con __init__.py
    # exclude=['test'] → Ignora cartelle di test
    packages=find_packages(exclude=['test']),
    
    # ===== DATA_FILES: FILE DA INSTALLARE (oltre al codice Python) =====
    # Ogni tupla: (destinazione_install, [lista_file_sorgente])
    # I file vengono copiati in install/share/multirobot_coordinator/
    data_files=[
        # --- 1. REGISTRAZIONE PACKAGE IN ROS2 ---
        # Installa file marker vuoto per ROS2 indexing
        # Permette a `ros2 pkg list` di trovare il package
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),  # File: resource/multirobot_coordinator (vuoto)
        
        # --- 2. METADATI ROS2 ---
        # Installa package.xml (contiene dipendenze e info package)
        ('share/' + package_name, ['package.xml']),
        
        # --- 3. LAUNCH FILES ---
        # Installa tutti i file .py nella cartella launch/
        # glob('launch/*.py') → trova bringup.launch.py
        # Destinazione: install/share/multirobot_coordinator/launch/
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
        # --- 4. FILE DI CONFIGURAZIONE ---
        # Installa tutti i file .yaml nella cartella config/
        # glob('config/*.yaml') → trova aruco_config.yaml, scanner_config.yaml
        # Destinazione: install/share/multirobot_coordinator/config/
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        
        # --- 5. MODELLI GAZEBO: MARKER ARUCO ID 5 (NORD) ---
        # Installa file di configurazione del modello Gazebo
        # model.config = metadati, model.sdf = descrizione 3D
        (os.path.join('share', package_name, 'gazebo', 'models', 'aruco_tag_5'), 
            ['gazebo/models/aruco_tag_5/model.config', 
             'gazebo/models/aruco_tag_5/model.sdf']),
        
        # Installa texture PNG del marker (immagine ArUco stampata sul modello)
        (os.path.join('share', package_name, 'gazebo', 'models', 'aruco_tag_5', 'materials', 'textures'), 
            ['gazebo/models/aruco_tag_5/materials/textures/marker_5.png']),
        
        # --- 6. MODELLI GAZEBO: MARKER ARUCO ID 10 (EST) ---
        (os.path.join('share', package_name, 'gazebo', 'models', 'aruco_tag_10'), 
            ['gazebo/models/aruco_tag_10/model.config', 
             'gazebo/models/aruco_tag_10/model.sdf']),
        
        (os.path.join('share', package_name, 'gazebo', 'models', 'aruco_tag_10', 'materials', 'textures'), 
            ['gazebo/models/aruco_tag_10/materials/textures/marker_10.png']),
        
        # NOTA: Marker 15 (Sud) e 20 (Ovest) hanno righe simili non mostrate qui
        # In totale: 4 marker ArUco × 2 righe ciascuno = 8 righe di installazione modelli
    ],
    
    # ===== DIPENDENZE PYTHON =====
    # Librerie Python necessarie (setuptools è sempre richiesto)
    install_requires=['setuptools'],
    
    # Indica se il package può essere zippato (False per ROS2, meglio lasciare True)
    zip_safe=True,
    
    # ===== METADATI AUTORE =====
    maintainer='Francesco',
    maintainer_email='francesco@todo.todo',
    description='Coordinator for iiwa scanner and fra2mo coverage robot',
    license='Apache-2.0',
    
    # ===== TESTING =====
    # Framework di test da usare (pytest per Python)
    tests_require=['pytest'],
    
    # ===== ENTRY POINTS: CREAZIONE ESEGUIBILI ROS2 =====
    # Questa sezione trasforma file Python in comandi eseguibili
    # Ogni riga crea un comando che può essere chiamato con `ros2 run`
    entry_points={
        'console_scripts': [
            # FORMATO: 'nome_comando = package.modulo:funzione'
            
            # --- NODO 1: SCANNER BRACCIO IIWA ---
            # Comando: iiwa_scanner_node
            # File: multirobot_coordinator/iiwa_scanner_node.py
            # Funzione: main()
            # Uso: ros2 run multirobot_coordinator iiwa_scanner_node
            'iiwa_scanner_node = multirobot_coordinator.iiwa_scanner_node:main',
            
            # --- NODO 2: TASK ALLOCATOR (MAPPING ARUCO) ---
            # Comando: task_allocator_node
            # File: multirobot_coordinator/task_allocator_node.py
            # Funzione: main()
            # Uso: ros2 run multirobot_coordinator task_allocator_node
            'task_allocator_node = multirobot_coordinator.task_allocator_node:main',
            
            # --- NODO 3: EXECUTOR NAVIGAZIONE ---
            # Comando: fra2mo_executor_node
            # File: multirobot_coordinator/fra2mo_executor_node.py
            # Funzione: main()
            # Uso: ros2 run multirobot_coordinator fra2mo_executor_node
            'fra2mo_executor_node = multirobot_coordinator.fra2mo_executor_node:main',
        ],
    },
)

# ===== COSA SUCCEDE DURANTE `colcon build`? =====
#
# 1. colcon legge package.xml → Trova dipendenze (rclpy, aruco_msgs, ecc.)
# 2. colcon esegue setup.py → Installa file secondo data_files
# 3. setuptools crea script eseguibili da entry_points
# 4. File copiati in install/multirobot_coordinator/:
#    ├── lib/multirobot_coordinator/
#    │   ├── iiwa_scanner_node      ← Script eseguibile
#    │   ├── task_allocator_node
#    │   └── fra2mo_executor_node
#    └── share/multirobot_coordinator/
#        ├── launch/bringup.launch.py
#        ├── config/aruco_config.yaml
#        ├── config/scanner_config.yaml
#        └── gazebo/models/...
#
# 5. Dopo `source install/setup.bash`:
#    - Comandi disponibili: iiwa_scanner_node, task_allocator_node, fra2mo_executor_node
#    - get_package_share_directory('multirobot_coordinator') funziona
#    - ros2 pkg list mostra multirobot_coordinator

# ===== CONFRONTO CON C++ (CMakeLists.txt) =====
#
# Python (setup.py):                  C++ (CMakeLists.txt):
# - packages=find_packages()          - add_executable()
# - data_files=[(dest, src)]          - install(FILES ... DESTINATION ...)
# - entry_points={'console_scripts'}  - ament_target_dependencies()
# - Nessuna compilazione              - Compilazione con g++/clang
# - Interpretato runtime              - Binario precompilato

# ===== FINE FILE =====
