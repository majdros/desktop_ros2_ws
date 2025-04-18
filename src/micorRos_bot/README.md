# MicroROS Bot Projekt

Dieses Projekt implementiert die Steuerung eines zweirädrigen Roboters unter Verwendung von MicroROS auf 'PlatformIO' als Entwicklungsplattform. Die Hauptkomponenten umfassen die Motorsteuerung, Encoder-Interrupts, PID-Regelung und die Integration mit ROS für die Kommunikation und Steuerung.

## Usage

1. **Einrichtung**
Befolge das offizielle MicroROS-Tutorial [creating a first application with FreeRTOS](https://micro.ros.org/docs/tutorials/core/first_application_rtos/freertos/). Dieses Tutorial umfasst die Installation aller notwendigen Abhängigkeiten und die Einrichtung des MicroROS-Agenten auf dem Hostsystem. 

2. **Hinweis**
    - Dieses Projekt basiert auf **MicroROS**, das auf einem **ESP32** mit **PlatformIO** als Entwicklungsplattform läuft.
    - Es wird **keine klassische Firmware** benötigt, da MicroROS direkt auf dem ESP32 läuft.

3. **Starten das micro-ROS Projekt**
```bash
cd microros_ws/
source install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev [device]
```

## Hauptkomponenten

### Motorsteuerung
Die Motorsteuerung erfolgt über die Klasse `MotorController`, die in der Datei `MotorController.h` definiert ist. Diese Klasse ermöglicht die Steuerung der Motoren, das Auslesen der Encoder-Werte und die Implementierung der PID-Regelung.

### Encoder-Interrupts
Die Encoder-Interrupts werden in der `setup`-Funktion in `main.cpp` konfiguriert. Diese Interrupts ermöglichen das präzise Auslesen der Encoder-Werte, die zur Berechnung der Radgeschwindigkeit und -position verwendet werden.

### PID-Regelung
Die PID-Regelung wird ebenfalls in der Klasse `MotorController` implementiert. Die PID-Parameter werden in der `setup`-Funktion initialisiert, um eine präzise Geschwindigkeitsregelung der Motoren zu ermöglichen.

### ROS-Integration
Die ROS-Integration erfolgt über die Bibliothek `micro_ros_arduino`. Die Funktionen `setupROS` und `spinROS` in `ros_interface.h` ermöglichen die Initialisierung und das kontinuierliche Ausführen der ROS-Kommunikation.


#### Funktionen


| Funktion                                                                     | Beschreibung                                                                                                                                                                                                                                                                                                                      |
|------------------------------------------------------------------------------|-----------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------------|
| `void setupROS()`                                                            | Initialisiert die micro-ROS-Infrastruktur, erstellt und konfiguriert die benötigten Publisher (z. B. `/wheel_odom`, `/joint_states`, `/tf_static`), Subscriber (z. B. `/cmd_vel_stamped`) und Timer-Callbacks.                                                                                                                                  |
| `void spinROS()`                                                             | Führt den micro-ROS Spin-Mechanismus aus, um eingehende Nachrichten zu verarbeiten und periodische Tasks (z. B. Motorsteuerung, Odometrie-Berechnung) aufzurufen. Läuft typischerweise in einer Schleife, bis der Knoten beendet wird.                                                                                             |
| `void cmdVelStampedCallback(const void* msgIn)`                              | Callback-Funktion, die aufgerufen wird, wenn eine neue `geometry_msgs::msg::TwistStamped` Nachricht auf dem `/cmd_vel_stamped` Topic empfangen wird. Extrahiert daraus die gewünschten Linear- und Rotationsgeschwindigkeiten und speichert sie, damit die Motorsteuerung diese Vorgaben umsetzen kann.                          |
| `void motorControlCallback(rcl_timer_t* timer, int64_t last_call_time)`      | Timer-Callback-Funktion, die in regelmäßigen Abständen aufgerufen wird, um die Motorsteuerung durchzuführen. Liest die aktuellen Encoderwerte aus, berechnet die erforderlichen Korrekturen (mithilfe einer PID-Regelung) und steuert die Motoren an, um die in `cmdVelStampedCallback` gesetzten Geschwindigkeiten zu erreichen. |
| `void publishOdom()`                                                         | Berechnet die Odometrie basierend auf den Encoder-Daten (Position und Orientierung) und publiziert sie in Form einer `nav_msgs::msg::Odometry`-Nachricht auf dem `/wheel_odom` Topic. Die Berechnung umfasst die Integration von Wegstrecken, Radabständen.                 |
| `void publishJointStates(float currentRpmL, float currentRpmR)`              | Erstellt eine `sensor_msgs::msg::JointState`-Nachricht mit den aktuellen Gelenkzuständen (z. B. Drehgeschwindigkeit oder Position der Räder) und publiziert diese auf dem `/joint_states` Topic.                                                                                                                                  |
| `void publishTf()`                                                           | Publiziert die relevanten Transformationen zwischen den Koordinatenrahmen (z. B. `base_link`, `odom`) auf dem `/tf_static` Topic. Dabei wird üblicherweise `tf2_msgs::msg::TFMessage` verwendet. Je nach Konfiguration kann hier auch die QoS-Einstellung `TRANSIENT_LOCAL` genutzt werden.                                |
| `void updateEncoderL()`                                                      | Interrupt Service Routine (ISR) für den linken Encoder. Aktualisiert die interne Zählvariable für den linken Radantrieb, indem sie bei jedem auftretenden Signal (abhängig von Drehrichtung) inkrementiert oder dekrementiert.                                                                                                    |
| `void updateEncoderR()`                                                      | Interrupt Service Routine (ISR) für den rechten Encoder. Aktualisiert die interne Zählvariable für den rechten Radantrieb, indem sie bei jedem auftretenden Signal (abhängig von Drehrichtung) inkrementiert oder dekrementiert.                                                                                                   |
| `struct timespec getTime()`                                                  | Synchronisiert die Zeit mit ROS und gibt die aktuelle Zeit als `timespec` zurück.                                                                                                                                                                                     |
| `bool syncTime()`                                                            | Synchronisiert die lokale Zeitbasis mit der Zeit, die vom ROS2-System vorgegeben wird, damit Zeitstempel in Nachrichten und Transformationsberechnungen konsistent bleiben. Gibt `true` zurück, wenn die Synchronisation erfolgreich war, sonst `false`.                                         |



## Dateien
### `src/main.cpp`
Diese Datei enthält die Hauptlogik des Projekts, einschließlich der Initialisierung der Motorcontroller, der Konfiguration der Encoder-Interrupts, der PWM-Konfiguration und der ROS-Initialisierung.


### `include/MotorController.h`
Diese Datei definiert die Klasse `MotorController`, die die Steuerung der Motoren, das Auslesen der Encoder-Werte und die Implementierung der PID-Regelung ermöglicht.

### `include/ros_interface.h`
Diese Datei enthält die Deklarationen für die ROS-Integration, einschließlich der Initialisierung und des Spin-Mechanismus sowie der Callback-Funktionen für die abonnierten und publizierten Topics.

### `include/globals.h`
Diese Datei enthält globale Variablen und Konstanten, die im gesamten Projekt verwendet werden, wie die Motorcontroller-Objekte, Roboterparameter und PWM-Parameter.
