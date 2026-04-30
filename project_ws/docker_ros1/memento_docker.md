# Fiche-mémoire Docker : concepts, vocabulaire, commandes essentielles

## 🧠 Concepts & vocabulaire importants

| Terme | Définition / rôle |
|---|---|
| **Image** | Une image Docker est un **modèle en lecture seule** ("template") contenant tout ce qu'il faut pour exécuter une application : système de base, dépendances, bibliothèques, configuration, etc. |
| **Conteneur (container)** | Une conteneur est une **instance vivante** d'une image : c'est l'environnement isolé dans lequel l'application tourne. On peut démarrer, arrêter, supprimer un conteneur sans modifier l'image. |
| **Dockerfile** | Fichier texte contenant des instructions pour construire une image : base (`FROM`), commandes d'installation (`RUN`), configuration d'environnement, dossier de travail (`WORKDIR`), etc. |
| **Build** | L'action de créer une image à partir d'un Dockerfile + le contexte (fichiers, dossier). |
| **Registry / Hub** | Dépôt (local ou distant) d'images : on peut y **télécharger** (pull) des images existantes, ou **envoyer** (push) des images que l'on a construites. |
| **Isolation & conteneurisation** | Contrairement à une machine virtuelle classique, un conteneur partage le noyau de l'OS hôte, mais garde un environnement isolé : système de fichiers, réseau, processus, ressources — ce qui rend l'approche plus légère et rapide. |
| **Volumes / montages / partage de ressources** | Mécanisme permettant de partager des fichiers, dossiers (ou d'autres ressources) entre l'hôte et le conteneur, ou entre conteneurs — utile pour la persistance des données ou l'accès à des ressources externes. |
| **Fichier `.env`** | Fichier texte à la racine du projet contenant des variables d'environnement sous la forme `CLE="valeur"`. Lu automatiquement par `docker compose` au démarrage des conteneurs. Pratique pour stocker des secrets (tokens, mots de passe) hors du code. À ajouter au `.gitignore` pour ne pas publier les secrets. |
| **Docker Compose** | Outil qui permet de définir et lancer **plusieurs conteneurs** ensemble via un fichier de configuration `docker-compose.yml`. Évite d'avoir à taper de longues commandes `docker run` manuelles. |

---

## 🔧 Commandes Docker de base (et options importantes)

| Commande | Usage & explication |
|---|---|
| `docker --version` / `docker info` / `docker help` | Connaître la version de Docker, obtenir des informations système, ou afficher l'aide. |
| `docker build -t <image:tag> .` | Construire une image à partir d'un Dockerfile dans le dossier courant. L'option `-t` permet de donner un nom (et un tag / version) à l'image. |
| `docker build --no-cache .` | Construire une image sans utiliser le cache — utile pour forcer la réinstallation de tout. |
| `docker images` | Lister toutes les images disponibles localement. |
| `docker rmi <image>` | Supprimer une image locale (utile si elle n'est plus utilisée). |
| `docker run [OPTIONS] IMAGE [COMMAND] [ARG …]` | Créer + démarrer un conteneur à partir d'une image. On peut aussi spécifier la commande à exécuter dans le conteneur. |
| Quelques options utiles de `docker run` : |  |
| `-d` | Détacher le conteneur — il tourne en arrière-plan (daemon). |
| `--name <nom>` | Donner un nom personnalisé au conteneur, pour le référencer plus facilement ensuite. |
| `-p <hôte:conteneur>` | Mapper / exposer un port du conteneur vers un port de l'hôte (utile pour services web, base de données…). |
| `-v <hôte>:<conteneur>` | Monter un volume / dossier de l'hôte dans le conteneur — pour partager des fichiers ou garder des données persistantes. |
| `docker ps` | Lister les conteneurs **en cours d'exécution**. |
| `docker ps -a` | Lister **tous** les conteneurs — qu'ils soient en cours ou arrêtés. |
| `docker start <container>` / `docker stop <container>` | Démarrer ou arrêter un conteneur existant (créé précédemment). `stop` met en pause le conteneur sans le détruire. |
| `docker exec -it <container> <commande>` | Exécuter une commande à l'intérieur d'un conteneur en cours d'exécution — par exemple ouvrir un shell (`bash` ou `sh`). |
| `docker logs <container>` | Afficher les journaux (logs) d'un conteneur : utile pour voir ce qu'il fait, debug, erreurs, sortie console. |
| `docker logs -f <container>` | Suivre les logs en temps réel (comme `tail -f`). Le `-f` signifie "follow". |
| `docker rm <container>` | Supprimer un conteneur arrêté (nettoyage). |
| `docker inspect <container|image>` | Obtenir des informations détaillées (configuration, métadonnées, volumes, réseau…) sur une image ou un conteneur. |

---

## 🐙 Docker Compose : commandes essentielles

`docker compose` (ou `docker-compose` dans l'ancienne version v1) gère plusieurs conteneurs définis dans un fichier `docker-compose.yml`. Toutes ces commandes se lancent **dans le dossier qui contient le `docker-compose.yml`**.

| Commande | Usage & explication |
|---|---|
| `docker compose build` | Construit (ou reconstruit) les images définies dans `docker-compose.yml`. |
| `docker compose build --no-cache` | Reconstruit les images sans utiliser le cache (force la réinstallation complète). |
| `docker compose up` | **Crée et démarre** les conteneurs. Lit automatiquement le fichier `.env` et injecte les variables dans les conteneurs. Reste accroché au terminal et affiche les logs. |
| `docker compose up -d` | Idem, mais en mode **détaché** (`-d` = "detached") : les conteneurs tournent en arrière-plan et le terminal est libéré. C'est l'usage normal. |
| `docker compose down` | **Arrête ET supprime** les conteneurs, ainsi que le réseau créé. ⚠️ Différent de `stop` : `down` détruit, `stop` met juste en pause. Les images et volumes sont conservés par défaut. |
| `docker compose stop` | Arrête les conteneurs sans les détruire (ils peuvent être redémarrés avec `start`). |
| `docker compose start` | Redémarre des conteneurs arrêtés (déjà créés). |
| `docker compose restart` | Redémarre les conteneurs (équivaut à `stop` + `start`). |
| `docker compose ps` | Liste les conteneurs gérés par le projet Compose courant. |
| `docker compose logs` | Affiche les logs des services. |
| `docker compose logs -f` | Suit les logs en temps réel. |
| `docker compose exec <service> <commande>` | Exécute une commande dans un conteneur en cours (équivalent de `docker exec`). |

### 🔄 Recette typique : recharger un changement de configuration `.env`

Quand on modifie le fichier `.env` après que le conteneur ait déjà démarré, il faut le recréer pour que les nouvelles variables soient injectées :

```bash
docker compose down      # détruit les conteneurs (mais garde l'image)
docker compose up -d     # recrée les conteneurs en arrière-plan, en relisant .env
```

Pas besoin de rebuild l'image — c'est l'affaire de quelques secondes.

### 📌 À noter : `docker compose` vs `docker-compose`

- `docker-compose` (avec un tiret) = ancienne version v1, binaire Python séparé.
- `docker compose` (avec un espace) = nouvelle version v2, intégrée à Docker.

Sur les installations récentes, c'est **`docker compose`** qui est recommandé.

---

## 🛠️ Concepts avancés / bonnes pratiques & précautions

- 🔄 **Isolation légère** : les conteneurs ne sont **pas** des machines virtuelles — ils sont beaucoup plus légers (mémoire, ressources, démarrage rapide) parce qu'ils partagent le noyau de l'OS hôte.
- 📦 **Portabilité** : une image Docker rend votre application **portable** : elle contient tout ce dont l'application a besoin, ce qui permet de l'exécuter sur différents serveurs ou machines, sans souci de configuration.
- 📁 **Volumes & persistance** : si votre application stocke des données (fichiers, logs, bases…), il est préférable d'utiliser un volume — sinon, les données risquent d'être perdues quand le conteneur est supprimé.
- ⚠️ **Sécurité & fiabilité des images** : les images peuvent contenir des vulnérabilités ou des données sensibles (clés, mots de passe…) — il vaut mieux utiliser des images "officielles" ou de confiance, et éviter d'inclure des secrets dans l'image.
- 🔐 **Secrets et fichier `.env`** : ne jamais inclure de tokens / mots de passe directement dans le `Dockerfile` ou dans une image. Utiliser un fichier `.env` à la racine du projet, et **toujours l'ajouter au `.gitignore`** pour ne pas le pousser sur GitHub.
- 🧹 **Nettoyage** : supprimer régulièrement les conteneurs et images inutilisés pour libérer de l'espace disque et éviter l'encombrement. Utiliser `docker rm`, `docker rmi` si nécessaire.

---

## 📋 Résumé « fiche de poche »

- Maîtriser les **concepts** : image ↔ conteneur, Dockerfile, build, run, volumes/ports, isolation, fichier `.env`.
- Connaître les **commandes Docker essentielles** : `docker build`, `docker run`, `docker ps`, `docker exec`, `docker stop/start`, `docker rm`, `docker images`, `docker rmi`, `docker logs`, `docker inspect`.
- Connaître les **commandes Docker Compose** : `docker compose build`, `docker compose up -d`, `docker compose down`, `docker compose logs -f`, `docker compose exec`.
- Appliquer les **bonnes pratiques** : utiliser des images fiables, monter des volumes pour la persistance, nettoyer les anciens conteneurs/images, éviter d'inclure des secrets dans les images (passer par `.env`).


xhost +local:docker

