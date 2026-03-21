## Installation de Nix (pour Linux)

#### Option 1
Lien pour le script d'installation officiel de Nix: https://nixos.org/download/

Je recommande d'installer la version Single-User si vous n'avez pas besoin d'installer Nix pour plusieurs utilisateurs.

1) Éxecuter le script d'installation de Nix.
2) Ajouter la configuration de Nix suivante si elle n'est pas dans ``nix.conf``.
** Il est possible que vous ayez à créer le fichier de configuration **
- Chemin pour l'installation Single-User: ``$HOME/.config/nix/nix.conf``
- Chemin pour l'installation Multi-User: ``/etc/nix/nix.conf``
Configuration: 
```
experimental-features = nix-command flakes
build-users-group =
sandbox = true
```

3) Aller dans le dossier de POLYSTAR-TAPROOT et exécuter la commande suivante:
```
nix develop
```
Vous devriez maintenant voir les dépendances du projet s'installer.
Vous aurez à exécuter cette commande à chaque ouverture du projet dans un terminal pour accéder aux dépendances.

#### Option 2 (Nécessaire pour Fedora avec SELinux)
Nix-Community: https://nix-community.github.io/nix-installers/

1) Choisissez votre installer en fonction de votre distribution:
- Pour Debian et Ubuntu: ``nix-multi-user-{version}.deb``
- Pour Fedora: ``nix-multi-user-{version}.rpm``
- Pour Arch: ``nix-multi-user-{version}.pkg.tar.zst`

2) Installer le package avec votre package manager:

- Debian et Ubuntu:
```
sudo apt install nix-multi-user-{version}.deb
```

- Fedora:
```
sudo dnf install nix-multi-user-{version}.rpm
```

- Arch: (à confirmer)
```
sudo pacman -Syu nix-multi-user-{version}.pkg.tar.zst
```


3) Ajouter la configuration de Nix suivante si elle n'est pas dans ``nix.conf``.
** Il est possible que vous ayez à créer le fichier de configuration **
- Chemin pour l'installation Multi-User: ``/etc/nix/nix.conf``
Configuration: 
```
experimental-features = nix-command flakes
build-users-group =
sandbox = true
```

4) Aller dans le dossier de POLYSTAR-TAPROOT et exécuter la commande suivante:
```
nix develop
```
Vous devriez maintenant voir les dépendances du projet s'installer.
Vous aurez à exécuter cette commande à chaque ouverture du projet dans un terminal pour accéder aux dépendances.
