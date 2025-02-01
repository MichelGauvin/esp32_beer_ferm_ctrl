Pour modulariser le code et séparer la gestion des préférences dans des fichiers `.h` et `.cpp`, vous pouvez créer un fichier d'en-tête pour déclarer les fonctions et une implémentation dans le fichier `.cpp`. Voici comment organiser le code :

### 1. Créer un fichier `PreferencesManager.h`

Dans ce fichier, vous déclarez les fonctions et les variables globales nécessaires.

```cpp
#ifndef PREFERENCESMANAGER_H
#define PREFERENCESMANAGER_H

#include <Preferences.h>

class PreferencesManager {
public:
  PreferencesManager();
  ~PreferencesManager();

  void begin();
  double getSetpoint(int index);
  bool getFermStatus(int index);
  int saveSetpoint(int index, double value);
  int saveFermStatus(int index, bool value);

private:
  Preferences preferences;
  static const char* SETPOINT_KEYS[4];
  static const char* FERMSTATUS_KEYS[4];
  static const char* PREF_NAMESPACE;
};

#endif
```

### 2. Créer un fichier `PreferencesManager.cpp`

Dans ce fichier, vous implémentez les fonctions définies dans le fichier d'en-tête.

```cpp
#include "PreferencesManager.h"

// Définition des clés et du namespace
const char* PreferencesManager::SETPOINT_KEYS[4] = { "setpoint1", "setpoint2", "setpoint3", "setpoint4" };
const char* PreferencesManager::FERMSTATUS_KEYS[4] = { "fermStatus1", "fermStatus2", "fermStatus3", "fermStatus4" };
const char* PreferencesManager::PREF_NAMESPACE = "ferment";

PreferencesManager::PreferencesManager() {
  // Constructeur, aucune initialisation spécifique ici
}

PreferencesManager::~PreferencesManager() {
  // Destruction, peut-être fermer les préférences si nécessaire
}

void PreferencesManager::begin() {
  preferences.begin(PREF_NAMESPACE, false);  // Initialisation des préférences
}

double PreferencesManager::getSetpoint(int index) {
  return preferences.getDouble(SETPOINT_KEYS[index], 20.0);  // Valeur par défaut 20.0
}

bool PreferencesManager::getFermStatus(int index) {
  return preferences.getBool(FERMSTATUS_KEYS[index], false);  // Valeur par défaut false
}

int PreferencesManager::saveSetpoint(int index, double value) {
  if (index < 0 || index >= 4) {
    Serial.println("Erreur : index de setpoint invalide");
    return -1;  // Erreur
  }
  if (isnan(value)) {
    Serial.println("Erreur : valeur invalide pour le setpoint");
    return -1;  // Erreur
  }
  preferences.putDouble(SETPOINT_KEYS[index], value);
  return 0;
}

int PreferencesManager::saveFermStatus(int index, bool value) {
  if (index < 0 || index >= 4) {
    Serial.println("Erreur : index de setpoint invalide");
    return -1;  // Erreur
  }
  preferences.putBool(FERMSTATUS_KEYS[index], value);
  return 0;
}
```

### 3. Modifier le fichier `main.ino`

Dans votre fichier principal `main.ino` ou un autre fichier `.cpp`, vous devez inclure le fichier d'en-tête `PreferencesManager.h` et utiliser l'objet de la classe `PreferencesManager`.

```cpp
#include <Arduino.h>
#include "PreferencesManager.h"

// Créer un objet PreferencesManager
PreferencesManager preferencesManager;

void setup() {
  // Initialisation de la communication série
  Serial.begin(115200);
  Serial.println("Initialisation...");

  // Initialiser les préférences
  preferencesManager.begin();

  // Exemple d'utilisation des préférences
  double setpoint1 = preferencesManager.getSetpoint(0);  // Récupérer le setpoint pour le premier fermenteur
  Serial.print("Setpoint 1 : ");
  Serial.println(setpoint1);

  bool fermStatus1 = preferencesManager.getFermStatus(0);  // Récupérer le statut pour le premier fermenteur
  Serial.print("Statut Ferm 1 : ");
  Serial.println(fermStatus1 ? "ON" : "OFF");

  // Sauvegarder une nouvelle valeur de setpoint
  preferencesManager.saveSetpoint(0, 25.0);  // Mettre à jour le setpoint pour le premier fermenteur
  preferencesManager.saveFermStatus(0, true);  // Mettre à jour le statut du premier fermenteur

  // Lire à nouveau pour vérifier que la sauvegarde a réussi
  setpoint1 = preferencesManager.getSetpoint(0);
  fermStatus1 = preferencesManager.getFermStatus(0);

  Serial.print("Nouveau Setpoint 1 : ");
  Serial.println(setpoint1);
  Serial.print("Nouveau Statut Ferm 1 : ");
  Serial.println(fermStatus1 ? "ON" : "OFF");
}

void loop() {
  // Le code du loop ici...
}
```

### Résumé :

1. **Fichier `PreferencesManager.h`** : Déclare la classe `PreferencesManager` et ses méthodes pour gérer les préférences.
2. **Fichier `PreferencesManager.cpp`** : Implémente les méthodes déclarées dans le fichier d'en-tête pour interagir avec les préférences.
3. **Fichier principal** (`main.ino` ou `.cpp`) : Utilise la classe `PreferencesManager` pour gérer les préférences de manière modulaire.

Cette approche vous permet de séparer clairement la logique de gestion des préférences et d’améliorer la lisibilité et la maintenabilité de votre code.