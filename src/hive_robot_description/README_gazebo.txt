# Workflow : Générer un modèle pour Gazebo Harmonic (gz sim)

## 1. Génération URDF à partir du Xacro

Se placer dans le dossier urdf :
    cd urdf

Générer le URDF depuis le fichier Xacro :
    xacro mk5_200.urdf.xacro > mk5_200.urdf

## 2. Conversion du URDF en SDF (pour Gazebo Harmonic)

Toujours dans le dossier urdf :
    gz sdf -p mk5_200.urdf > mk5_200.sdf

## 3. Lancement dans Gazebo Harmonic (gz sim)

    gz sim mk5_200.sdf

---

# Notes :
- Modifier uniquement le fichier Xacro (`mk5_200.urdf.xacro`) pour toutes les évolutions du robot.
- Vérifier la sortie du URDF en cas d’erreur (fichier vide = erreur de xacro).
- Le SDF généré est compatible Gazebo Harmonic (gz sim).
- Penser à ajouter les plugins Gazebo (diff_drive, sensors…) dans `mk5_200.gazebo.xacro` pour la simulation.
- Le URDF sert à la visualisation et à robot_state_publisher (RViz).

---

Fichier à garder dans le package pour toute l’équipe Hive Robotics.
