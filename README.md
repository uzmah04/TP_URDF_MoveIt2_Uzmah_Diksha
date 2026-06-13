# Robotic Arm - Modélisation URDF/Xacro & MoveIt 2

## 1. Présentation du robot et description du travail réalisé

Ce dépôt contient la modélisation complète d'un bras robotique à 6 degrés de liberté (DDL) + pince parallèle, conçu pour des applications de manipulation légère. Le robot se compose des éléments suivants :

- **Base fixe** (base_link, base_plate)
- **Bras inférieur moteur** (forward_drive_arm)
- **Bras horizontal** (horizontal_arm)
- **Support de pince** (claw_support)
- **Mécanisme triangulaire** (triangular_link)
- **Bras vertical moteur** (vertical_drive_arm)
- **Trois liaisons intermédiaires** (link, link2, link3)
- **Deux platines de servo-moteurs** (servo_plate, servo_plate2)
- **Pince parallèle** (right_finger, left_finger)


## 2. Justification du choix URDF vs Xacro

### 2.1 Critère ayant motivé la décision

Le choix s'est porté sur **Xacro** plutôt qu'un fichier URDF unique pour les raisons suivantes, classées par ordre de priorité :

| Critère | Impact |
|---------|--------|
| **Réutilisabilité** | La constante `PI` est définie une seule fois dans `common_properties.xacro` et réutilisée partout |
| **Lisibilité** | Le fichier principal `robotic_arm.urdf.xacro` reste épuré (3 lignes d'includes) |
| **Maintenabilité** | Les matériaux (grey, blue) sont centralisés ; une modification impacte tous les liens |
| **Complexité maîtrisée** | Le robot possède 15 liens et 16 joints, une séparation modulaire est indispensable |


### 2.2 Dans quel cas auriez-vous fait un choix différent ?

L'approche URDF uniquement (sans Xacro) aurait été privilégiée dans les cas suivants :

| Cas | Raison |
|-----|--------|
| **Robot simple avec moins de 5 liens** | La surcharge d'organisation d'un projet Xacro (fichiers multiples, includes) n'est pas justifiée pour un petit robot |
| **Absence de répétition dans les propriétés** | Si chaque lien a des matériaux, masses et géométries uniques, l'intérêt des macros diminue fortement |
| **Projet pédagogique pour débutants** | L'étape de prétraitement `xacro → URDF` ajoute une complexité conceptuelle supplémentaire pour des apprenants |
| **Débogage fréquent** | Les erreurs sont plus faciles à localiser dans un seul fichier URDF que dans l'URDF généré après expansion Xacro |
| **Intégration dans un outil ne supportant pas Xacro** | Certains anciens outils ou visualiseurs ne comprennent que le format URDF pur sans prétraitement |
| **Prototypage rapide** | Pour une preuve de concept à itérer rapidement, maintenir un seul fichier est plus efficace |
| **Contrainte de dépendances minimales** | Xacro nécessite un package supplémentaire ; un URDF pur ne dépend que de la bibliothèque URDF standard |

---

## 3. Hypothèses faites sur les masses, inerties et limites articulaires

### 3.1 Masses et inerties

Les masses et les tenseurs d'inertie ont été déterminés par une approche de modélisation rigoureuse sur OnShape :

1. **Importation des fichiers STL** : Chaque mesh fourni (`basement.STL`, `base_plate.STL`, `forward_drive_arm.STL`, etc.) a été importé dans le logiciel Onshape

2. **Attribution d'un matériau** : Nous avons supposé que le robot est fabriqué en **aluminium 6061**, un alliage largement utilisé en robotique pour son excellent rapport résistance/poids, sa facilité d'usinage et sa faible inertie

3. **Calcul automatique** : Onshape a calculé automatiquement la masse et la matrice d'inertie pour chaque pièce en fonction du volume et de la densité de l'aluminium 6061 (≈ 2700 kg/m³)

4. **Conversion des unités** : Les valeurs extraites d'Onshape étaient exprimées en grammes (g) pour la masse et en millimètres (mm) pour les dimensions du tenseur d'inertie. Elles ont été converties dans le Système International (kg et m) car Xacro utilise les unités SI (kg, m, s, rad)

5. **Intégration** : Les valeurs convertées ont été directement reportées dans les balises `<inertial>` de chaque link

**Valeurs obtenues :**

| Link | Masse (kg) | Remarque |
|------|------------|----------|
| `base_link` | 0.115956 | Pièce la plus massive (stabilité de la base) |
| `base_plate` | 0.118660 | Plaque support du mécanisme |
| `forward_drive_arm` | 0.035532 | Bras moteur principal |
| `horizontal_arm` | 0.023817 | Structure horizontale |
| `claw_support` | 0.018141 | Support de pince |
| `triangular_link` | 0.011737 | Liaison triangulaire légère |
| `link`, `link2`, `link3` | 0.007673 | Liens intermédiaires identiques |
| `vertical_drive_arm` | 0.007797 | Bras vertical |
| `servo_plate`, `servo_plate2` | 0.003430 | Platines de fixation servo |
| `right_finger` | 0.003984 | Doigt droit de la pince |
| `left_finger` | 0.003779 | Doigt gauche de la pince |

**Hypothèse sous-jacente :**

L'utilisation de l'aluminium 6061 pour toutes les pièces est une simplification raisonnable pour la simulation. Dans un robot réel, certains éléments (vis, roulements, moteurs) auraient des densités différentes.

### 3.2 Limites articulaires (joint limits)

Les limites articulaires ont été déterminées empiriquement par visualisation dans RViz :

1. **Configuration initiale** : Des valeurs provisoires ont été définies pour chaque joint de type `revolute`

2. **Simulation interactive** : Le `joint_state_publisher_gui` a été utilisé pour actionner manuellement chaque articulation

3. **Détection des collisions** : En faisant varier chaque joint sur toute sa plage possible, nous avons identifié les angles provoquant des interpénétrations entre les mailles

4. **Ajustement itératif** : Les limites ont été progressivement resserrées jusqu'à éliminer les collisions, tout en préservant un maximum d'amplitude de mouvement

**Valeurs retenues :**

| Joint | lower (rad) | upper (rad) | lower (°) | upper (°) | Justification |
|-------|-------------|-------------|-----------|-----------|---------------|
| `forward_drive_arm_to_horizontal_arm` | -0.6796 | 0.3325 | -38.9 | +19.0 | Évitement collision avec la `base_plate` et le mécanisme triangulaire |
| `forward_drive_arm_to_triangular_link` | -0.4743 | 0.2661 | -27.2 | +15.2 | Course limitée par l'encombrement du servo et des liaisons |
| `triangular_link_to_link` | -2.094 | 2.094 | -120 | +120 | Grande amplitude possible grâce à la conception du mécanisme |
| `base_plate_to_vertical_drive_arm` | 0 | -0.1967 | 0 | -11.3 | Rotation inversée (contrainte géométrique du prototype) |
| `base_plate_to_link3` | -1.094 | 1.094 | -62.7 | +62.7 | Évitement de la `base_plate` et du `forward_drive_arm` |
| `claw_support_to_right_finger` | -0.4500 | 0.000 | -25.8 | 0 | Ouverture maximale de la pince |
| `claw_support_to_left_finger` | -0.4500 | 0.000 | -25.8 | 0 | Mimic du doigt droit, mouvement symétrique inverse |


### 3.3 Paramètres d'inertie

Les matrices d'inertie suivent le format standard :
ixx ixy ixz
ixy iyy iyz
ixz iyz izz


Les termes extra-diagonaux (ixy, ixz, iyz) sont non-nuls pour refléter la géométrie asymétrique des pièces.

---

## 4. Instructions d'installation pas-à-pas

### 4.1 Prérequis

Ce guide suppose que vous avez déjà :

- **ROS 2 Jazzy** installé et configuré sur votre système (Ubuntu 24.04)
- **Un workspace ROS 2** déjà créé et fonctionnel (ex: `~/ros2_ws`)
- **Votre environnement ROS 2 sourcé** dans votre terminal

### 4.2 Installation des dépendances

Avant de compiler le package, installez les dépendances nécessaires :

```bash
# Xacro (généralement inclus avec ROS 2, mais vérifié)
sudo apt install ros-jazzy-xacro

# MoveIt 2
sudo apt install ros-jazzy-moveit

# Joint State Publisher GUI (pour contrôler manuellement les articulations)
sudo apt install ros-jazzy-joint-state-publisher-gui

# RViz 2 (visualisation)
sudo apt install ros-jazzy-rviz2
```

### 4.3 Clonage du dépôt

Cloner le dépôt
```bash
git clone https://github.com/uzmah04/TP_URDF_MoveIt2_Uzmah_Diksha.git
```

Placez le package dans le dossier `src` de votre workspace ROS 2 :
```bash
# Se rendre dans le dossier src de votre workspace
cd ~/ros2_ws/src

# Alternative : copie manuelle si vous avez téléchargé le dossier
# cp -r /chemin/vers/robotic_arm ~/ros2_ws/src/
```

### 4.4 Compilation du package
```bash
# Revenir à la racine du workspace
cd ~/ros2_ws

# Compiler le package
colcon build --packages-select robotic_arm

# Sourcer le workspace pour que ROS 2 reconnaisse le nouveau package
source ~/ros2_ws/install/setup.bash
```

## 5. Commandes pour lancer RViz 2 et MoveIt 2

### 5.1 Visualisation du robot dans RViz 2

Pour visualiser directement le robot dans RViz 2 avec la configuration prédéfinie :

```bash
cd ~/ros2_ws
ros2 launch robotic_arm display.launch.py
```

Cette commande lance RViz 2 avec le modèle du robot déjà chargé et affiché grâce à la config de Rviz déjà présente dans le dossier `config`. Vous pouvez visualiser l'intégralité du bras robotique avec ses meshes STL.

### 5.2 Lancement de MoveIt 2

Pour utiliser le pipeline de planification de trajectoires :
```bash

```

## 6. Captures d'écran

### 6.1 Modèle du robot dans RViz 2

> *À insérer : capture d'écran montrant le modèle complet du bras robotique dans RViz 2 avec tous les meshes STL chargés et affichés correctement.*

### 6.2 Visualisation des collisions (géométries simplifiées)

> *À insérer : capture d'écran montrant les géométries de collision (cylindres et boîtes) en mode wireframe ou transparent.*

### 6.3 Planification de trajectoire avec MoveIt 2

> *À insérer : capture d'écran ou GIF animé montrant l'interface MoveIt 2 dans RViz 2 avec :*
> - *Le robot dans sa configuration initiale*
> - *Un objectif de trajectoire défini (position cible de l'effecteur)*
> - *La trajectoire planifiée affichée en vert*
> - *Le robot en cours d'exécution ou ayant atteint la position cible*

### 6.4 Graphe des transformations TF

> *À insérer : capture d'écran de l'arbre TF généré par la commande `ros2 run tf2_tools view_frames` montrant toutes les frames du robot.*

## 7. Difficultés rencontrées et solutions apportées

### 7.1 Problème de scaling des meshes STL

| Problème | Solution |
|----------|----------|
| Les meshes apparaissaient énormes dans RViz 2 (environ 100 fois trop grands) | Ajout de l'attribut `scale="0.01 0.01 0.01"` sur chaque balise `<mesh>` dans tous les liens |

**Avant :**
```xml
<mesh filename="package://robotic_arm/meshes/basement.STL"/>
```
**Après :**
```xml
<mesh filename="package://robotic_arm/meshes/basement.STL" scale="0.01 0.01 0.01"/>
```

### 7.2 Orientation incorrecte des meshes

| Problème | Solution |
|----------|----------|
| Certains meshes apparaissaient désorientés dans RViz 2 (ex: `horizontal_arm`, `triangular_link`, `servo_plate`) | Ajustement des angles RPY dans les balises `<origin>` en utilisant la constante `${PI}` et ses dérivés (`${PI/2}`, `-${PI/2}`) |

**Exemple pour `horizontal_arm` :**
```xml
<origin rpy="${PI/2} 0 ${PI/2}" xyz="-0.03 -0.4 -0.06"/>
```

### 7.3 Crash du MoveIt Setup Assistant sous Wayland

| Problème | Solution |
|----------|----------|
| Le MoveIt Setup Assistant se fermait brutalement à environ 50 % du chargement après la sélection du fichier URDF/Xacro. Le robot était correctement parsé mais le rendu RViz intégré ne pouvait pas être créé sous une session Wayland. | Lancer le MoveIt Setup Assistant en forçant Qt à utiliser le backend X11/XCB : `QT_QPA_PLATFORM=xcb ros2 launch moveit_setup_assistant setup_assistant.launch.py` |

**Erreur observée :**
```
Invalid parentWindowHandle (wrong server or screen)

Unable to create the rendering window after 100 tries

rviz::RenderSystem: error creating render window
```
**Vérification du type de session :**
```bash
echo $XDG_SESSION_TYPE
```

**Résultat :**
```
wayland
```

**Commande utilisée avant correction :**
```bash
ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

Commande utilisée après correction :
```bash
QT_QPA_PLATFORM=xcb ros2 launch moveit_setup_assistant setup_assistant.launch.py
```

### 7.4 Paramètres de Masse et Inertie manquantes

| Problème | Solution |
|----------|----------|
| Avertissements sur l'absence d'inertie et mass dans le fichier STL | Import sur Onshape et assignement d'un matériel par hypothèse pour avoir une estimation du masse et inertie à fin de l'utiliser sur moveit |

### 7.5 Unités incorrectes pour les masses et inerties
| Problème | Solution |
|----------|----------|
| Onshape exporte les masses en grammes (g) et les inerties avec des dimensions en millimètres (mm), alors que Xacro utilise le Système International (kg, m) | Conversion systématique de toutes les valeurs : division par 1000 pour les masses, et division par 1 000 000 pour les termes d'inertie (car 1 mm = 0.001 m → mm² = 1e-6 m²) |

