# Enhancing Routing and Quality of Service Using AI-driven Technique for Internet of Vehicles Contexts

Projet édité par Oussama SENOUCI

Ce projet utilise les bibliothèques NS-3 et MOVE pour simuler des scénarios de réseau de véhicules (Internet of Vehicles). Le projet vise à améliorer l'acheminement et la qualité de service dans des contextes Internet of Vehicles en utilisant des techniques basées sur l'IA.

## Prérequis

- **OS** : Ubuntu 18.04 ou plus récent
- **Bonne maîtrise des commandes Linux**
- **NS-3** installé et configuré sur votre système.
- **MOVE** installé et configuré sur votre système.

## Répertoires

- **scratch** : Contient les fichiers NS-3 nécessaires pour simuler les scénarios Internet of Vehicles. Les scripts de simulation et les fichiers liés sont stockés ici.

- **MOVE** : Contient les fichiers de configuration de simulation pour simuler la mobilité. Vous y trouverez également les fichiers de trace de mobilité générés par MOVE.

- **results** : Stocke les résultats de simulation, tels que la durée de vie des clusters (CH lifetime et CM lifetime), le taux de livraison des paquets (Packet Delivery Ratio), et la surcharge de communication (Communication overhead).

## Étapes pour exécuter le projet

1. **Cloner le projet** : Clonez ce dépôt GitHub sur votre machine locale.

    ```bash
    git clone https://github.com/votre-utilisateur/github-repo.git
    ```

2. **Copier les fichiers** : Placez les fichiers de votre projet dans le répertoire `scratch` de NS-3.

    ```bash
    cp -r votre-projet/* /chemin/vers/ns3/scratch/
    ```

3. **Exécuter MOVE** : Utilisez MOVE pour créer des fichiers de mobilité à partir des fichiers de configuration de mobilité (XML) et de vos scénarios.

    ```bash
    java -jar move.jar /chemin/vers/configuration.xml
    ```

4. **Configurer la simulation** : Configurez les paramètres de la simulation dans les fichiers de configuration de votre simulation IoV.

    - Mettez à jour les paramètres tels que le nombre de véhicules, la durée de la simulation, et autres.

5. **Compiler la simulation** : Compilez le code de simulation NS-3 dans le répertoire `scratch`.

    ```bash
    cd /chemin/vers/ns3/
    ./waf
    ```

6. **Exécuter la simulation** : Exécutez la simulation à l'aide du fichier de démarrage de votre simulation IoV.

    ```bash
    ./waf --run scratch/My_Simulation-IoV
    ```

7. **Analyse des résultats** : Les résultats de la simulation sont généralement générés dans des fichiers de sortie (log, CSV, etc.). Analysez ces résultats pour évaluer les performances du réseau.

## Documentation

- Consultez la documentation officielle de [NS-3](https://www.nsnam.org/docs/) pour des informations détaillées sur l'utilisation de la bibliothèque NS-3.
- Consultez la documentation officielle de [MOVE](http://networks.cs.ucdavis.edu/move/) pour des informations détaillées sur l'utilisation de la bibliothèque MOVE.

## Contribution

Les contributions à ce projet sont les bienvenues. Veuillez soumettre vos demandes de fusion via des branches séparées et inclure des tests appropriés.







