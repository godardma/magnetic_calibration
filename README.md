# Magnetic calibration

Extracted from https://gitlab.ensta-bretagne.fr/lemezoth/voiture2A-ros

## Auteur :

:student: Maël GODARD <mael.godard@ensta-bretagne.org> (FISE 2023)

## Git Structure :

* :file_folder: [/helios_ros2](helios_ros2) : **dossier contenant les sources**
* :file_folder: [/launch](launch) : **dossier contenant les launcher**
* :file_folder: [/logs](logs) : **dossier contenant les logs de mission**
* :file_folder: [/path](path) : **dossier contenant les fichiers de path à suivre**
* :file_folder: [/rviz2_config](rviz2_config) : **dossier contenant la config rviz2 conseillée**
* :spiral_notepad: [package.xml](package.xml)
* :spiral_notepad: [setup.py](setup.py)    **fichier de setup ROS2 python**
* :spiral_notepad: [README.md](README.md)

## Technologies :

* Ubuntu 20.04
* Python
* ROS2 foxy


## Building the package

* Cloner le repo dans le dossier src d'un workspace ROS2 Foxy
* Se placer à la racine du workspace ROS2 Foxy
* Build le package :
```bash
colcon build --symlink-install --packages-select helios_ros2
. install/setup.bash
```

Le package étant en python, le paramètre --symlink-install permet de ne pas avoir à recompiler le package après un changement dans un des noeuds. 

## Lancement :
* Pour la simulation:
    ```bash
    ros2 launch helios_ros2 helios_simu.launch.py
    ```


* Sur le Helios :

    Pour se connecter au Helios via ssh, se connecter au reseau munu_ubnt (il sera nécessaire de se fixer une adresse IP de la forme 10.43.20.XXX), puis dans un terminal :

    ```bash
    ssh s100@10.43.20.223
    # mot de passe : s100
    ```

    Ouvrir le port du recepteur GNSS et envoyer les corrections RTK dans un terminal:

    ```bash
    narval_supply_control.py -e usbl
    send-rtk-corrections
    ```
    
    Dans un autre terminal sur le Helios:

    ```bash
    ros2 launch helios_ros2 helios.launch.py
    ```

* Visualisation

    Dans un terminal, lancer rviz2 et ouvrir la config jointe dans le dossier [/rviz2_config](rviz2_config) pour visualiser l'USV et les waypoints en temps réel


## Logs

Une écriture de logs est implémentée. Le nom du fichier peut être choisi depuis le launcher et le chemin à suivre et la trajectoire se retrouvent dans le dossier [logs](logs).

Pour exploiter ces logs, se placer dans le dossier helios_ros2 (à la racine du package) et exécuter la commande :
```bash
python3 helios_ros2/create_gpx.py logs_test
```
Où "logs_test" est le nom du fichier de logs choisi. Les fichiers de logs associés seront convertis du format txt au format gpx et pourront être affichés sur Géoportail.




## Notes

Les positions (Pose) sont données dans le repère Lambert93 à un offset près (voir point de référence dans le launch file)

Il est possible de préparer plusieurs missions à l'avance. Pour celà il suffit d'enregistrer les waypoints dans un fichier texte au format suivant:

* Un point par ligne
* Un point est défini par sa longitude et sa latitude (à Guerledan de l'ordre de -3° et 48°) , séparées par une virgule (et sans espace)
* Les longitudes et latitudes sont en degrés décimaux

Les fichiers sont ensuite à stocker dans le dossier [path](path) et leur nom est à renseigner sous "pathfile_name" dans le [launcher](launch) (un seul par launch)

En cas de difficulté à compiler ce repo sous ROS2 humble (pour une machine sous ubuntu 22.04 par exemple), il est conseillé d'utiliser un docker ROS Foxy comme indiqué [ici](https://hub.docker.com/_/ros/)
