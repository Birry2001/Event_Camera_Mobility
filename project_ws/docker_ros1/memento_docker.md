# Fiche-mémoire Docker : concepts, vocabulaire, commandes essentielles

## 🧠 Concepts & vocabulaire importants

| Terme | Définition / rôle |
|---|---|
| **Image** | Une image Docker est un **modèle en lecture seule** (“template”) contenant tout ce qu’il faut pour exécuter une application : système de base, dépendances, bibliothèques, configuration, etc. |
| **Conteneur (container)** | Une conteneur est une **instance vivante** d’une image : c’est l’environnement isolé dans lequel l’application tourne. On peut démarrer, arrêter, supprimer un conteneur sans modifier l’image. |
| **Dockerfile** | Fichier texte contenant des instructions pour construire une image : base (`FROM`), commandes d’installation (`RUN`), configuration d’environnement, dossier de travail (`WORKDIR`), etc. |
| **Build** | L’action de créer une image à partir d’un Dockerfile + le contexte (fichiers, dossier). |
| **Registry / Hub** | Dépôt (local ou distant) d’images : on peut y **télécharger** (pull) des images existantes, ou **envoyer** (push) des images que l’on a construites. |
| **Isolation & conteneurisation** | Contrairement à une machine virtuelle classique, un conteneur partage le noyau de l’OS hôte, mais garde un environnement isolé : système de fichiers, réseau, processus, ressources — ce qui rend l’approche plus légère et rapide. |
| **Volumes / montages / partage de ressources** | Mécanisme permettant de partager des fichiers, dossiers (ou d'autres ressources) entre l’hôte et le conteneur, ou entre conteneurs — utile pour la persistance des données ou l’accès à des ressources externes. |

---

## 🔧 Commandes Docker de base (et options importantes)

| Commande | Usage & explication |
|---|---|
| `docker --version` / `docker info` / `docker help` | Connaître la version de Docker, obtenir des informations système, ou afficher l’aide. |
| `docker build -t <image:tag> .` | Construire une image à partir d’un Dockerfile dans le dossier courant. L’option `-t` permet de donner un nom (et un tag / version) à l’image. |
| `docker build --no-cache .` | Construire une image sans utiliser le cache — utile pour forcer la réinstallation de tout. |
| `docker images` | Lister toutes les images disponibles localement. |
| `docker rmi <image>` | Supprimer une image locale (utile si elle n’est plus utilisée). |
| `docker run [OPTIONS] IMAGE [COMMAND] [ARG …]` | Créer + démarrer un conteneur à partir d’une image. On peut aussi spécifier la commande à exécuter dans le conteneur. |
| Quelques options utiles de `docker run` : |  |
| `-d` | Détacher le conteneur — il tourne en arrière-plan (daemon). |
| `--name <nom>` | Donner un nom personnalisé au conteneur, pour le référencer plus facilement ensuite. |
| `-p <hôte:conteneur>` | Mapper / exposer un port du conteneur vers un port de l’hôte (utile pour services web, base de données…). |
| `-v <hôte>:<conteneur>` | Monter un volume / dossier de l’hôte dans le conteneur — pour partager des fichiers ou garder des données persistantes. |
| `docker ps` | Lister les conteneurs **en cours d’exécution**. |
| `docker ps -a` | Lister **tous** les conteneurs — qu’ils soient en cours ou arrêtés. |
| `docker start <container>` / `docker stop <container>` | Démarrer ou arrêter un conteneur existant (créé précédemment). |
| `docker exec -it <container> <commande>` | Exécuter une commande à l’intérieur d’un conteneur en cours d’exécution — par exemple ouvrir un shell (`bash` ou `sh`). |
| `docker logs <container>` | Afficher les journaux (logs) d’un conteneur : utile pour voir ce qu’il fait, debug, erreurs, sortie console. |
| `docker rm <container>` | Supprimer un conteneur arrêté (nettoyage). |
| `docker inspect <container|image>` | Obtenir des informations détaillées (configuration, métadonnées, volumes, réseau…) sur une image ou un conteneur. |

---

## 🛠️ Concepts avancés / bonnes pratiques & précautions

- 🔄 **Isolation légère** : les conteneurs ne sont **pas** des machines virtuelles — ils sont beaucoup plus légers (mémoire, ressources, démarrage rapide) parce qu’ils partagent le noyau de l’OS hôte.  
- 📦 **Portabilité** : une image Docker rend votre application **portable** : elle contient tout ce dont l’application a besoin, ce qui permet de l’exécuter sur différents serveurs ou machines, sans souci de configuration.  
- 📁 **Volumes & persistance** : si votre application stocke des données (fichiers, logs, bases…), il est préférable d’utiliser un volume — sinon, les données risquent d’être perdues quand le conteneur est supprimé.  
- ⚠️ **Sécurité & fiabilité des images** : les images peuvent contenir des vulnérabilités ou des données sensibles (clés, mots de passe…) — il vaut mieux utiliser des images “officielles” ou de confiance, et éviter d’inclure des secrets dans l’image.  
- 🧹 **Nettoyage** : supprimer régulièrement les conteneurs et images inutilisés pour libérer de l’espace disque et éviter l’encombrement. Utiliser `docker rm`, `docker rmi` si nécessaire.  

---

## 📋 Résumé « fiche de poche »

- Maîtriser les **concepts** : image ↔ conteneur, Dockerfile, build, run, volumes/ports, isolation.  
- Connaître les **commandes essentielles** : `docker build`, `docker run`, `docker ps`, `docker exec`, `docker stop/start`, `docker rm`, `docker images`, `docker rmi`, `docker logs`, `docker inspect`.  
- Appliquer les **bonnes pratiques** : utiliser des images fiables, monter des volumes pour la persistance, nettoyer les anciens conteneurs/images, éviter d’inclure des secrets dans les images.  


xhost +local:docker

