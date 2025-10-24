# unitree_h1_visualization_ws

В данном репозитории лежат ROS2-пакеты, необходимые для визуализации движений робота Unitree H1.
Пакты которые задают движения робота см. [meta_launch_ws](https://github.com/cyberbanana777/unitree_h1_meta_launch_ws?tab=readme-ov-file#-какие-репозитории-установить).

## 🔖  Оглавление

1. [📦 Содержание репозитория](#-содержание-репозитория)
2. [🚀 Быстрый старт](#-быстрый-старт)
3. [⚙️ Предварительные требования](#️-предварительные-требования)
4. [🧪 Использование](#-использование)
   - 4.1 [Запуск узлов](#запуск-узлов)
   - 4.2 [Запуск launch-файлов](#запуск-launch-файлов)
5. [📚 Кастомные библиотеки](#-кастомные-библиотеки)
   - 5.1 [`rviz_util.py`](#rviz_utilpy)
6. [📡 Интерфейсы](#-интерфейсы-топики-сервисы-действия-параметры)
   - 6.1 [Пакет `h1_description`](#пакет-1-h1_description)
   - 6.2 [Пакет `h1_move_joint_rviz`](#пакет-2-h1_move_joint_rviz)
7. [🗺️ Архитектура](#️-архитектура)
   - 7.1 [`h1_description`](#h1_description-1)
   - 7.2 [`completed_scripts_visualization`](#completed_scripts_visualization)
8. [📜 Лицензия](#-лицензия)
9. [🙏 Благодарности](#-благодарности)
10. [💡 Предложения и корректировки](#-предложения-и-корректировки)

---

## 📦 Содержание репозитория
* **`completed_scripts_visualization/`**: Содержит python launch-файлы, которые запускают конфигурации нод из данного репозитория для задач соответствующих названиям launch-файлов.
* **`h1_description/`**: Содержит urdf-описание робота, launch-файлы, которые взаимодействуют с urdf-описанием робота.
*   **`h1_move_joint_rviz/`**: Содержит ноды, которые заставляют модель робота в rviz двигаться в соответствии с показаниями реальных или целевых положений суставов. Для первого варианта нужен робот, для второго - должна быть нода которая отправляет целевые положения для моторов ( публикующая в `/positions_to_unitree`).
*   **`docs/`**: Дополнительная документация.
*   **`.gitignore`**: Файлы, которые git не отслеживает (служебный файл) 
*   **`LICENSE`**: Лицензия, под которой распространяется данное программное обеспечение
*   **`README.md`**: Этот файл.
*   **`install_dependensies.bash`** - Bash-скрипт,  при запуске которого устанавливаются все необходимые python-зависимости через pip.

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🚀 Быстрый старт
Пошаговая инструкция для **быстрого** запуска основной функциональности. Предполагаем, что ROS2 (Foxy) уже установлен.
    
‼️**Репозиторий нужно устанавливать на локальной машине, а не удаленном компьютере робота, для быстрой и корректной работы графическго интерфейса.**

1.  **Клонировать репозиторий** в `src` вашего workspace:
```bash
mkdir -p unitree_h1_visualization_ws/src
cd unitree_h1_visualization_ws/src
git clone https://github.com/cyberbanana777/unitree_h1_visualization_ws.git .
my_pwd=$(pwd)
```
2. **Установить проприетарные зависимости** (по ссылкам инструкции по установке от производителя): 
-   [unitree_h1_control_ws](https://github.com/cyberbanana777/unitree_h1_control_ws) - Репозиторий, котоый нужно установить, в нем находится библиотека, содержащая наиболее часто используемые функции, словари и тд при работе с Unitree H1. Инструкция по установке есть внутри репозитория и также есть bash-скрипт, котоый полоностью устанавливает репозиторий и все зависммости и библиотеки. Находится он [здесь](https://github.com/cyberbanana777/unitree_h1_meta_launch_ws?tab=readme-ov-file#соло-скрипты). Подробнее читайте в инструкции к репозиториям.
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2). Если вы уже установили данную зависимость при установке другого репозитория, повторно выполнять не нужно. По ссылке выполняете всё, что написано до `Connect to Unitree robot`. После всех действий нужно выполнить команду:
```bash
echo "source ~/unitree_ros2/cyclonedds_ws/install/local_setup.bash" >> ~/.bashrc
source ~/.bashrc 
```
3. **Установить pip-зависимости:**
Для простоты мы сделали bash-скрипт, который автоматизирует этот процесс. Его необходимо запустить. Для этого выполним команды:
```bash
cd $my_pwd
chmod +x install_dependensies.bash
sudo ./install_dependensies.bash
```
4.  **Собрать workspace:**
```bash
cd $my_pwd/..
colcon build # --symlink-install позволяет не colcon'ить область после изменений кода пакетов, что бы изменения были видны системе.
source install/local_setup.bash  # Или setup.zsh - в зависимости от вашего интерпретатора командной строки
```
5. Добавить `source` в `~/.bashrc`: 
Зачем? - Чтобы не делать при каждом перезапуске терминала `source install/local_setup.bash`. Выполните команды ниже и это изменит код `~/.bashrc` - скрипта, который выполняется при запуске нового терминала. Для каждой ws делается отдельно. 
```bash
line_to_add="source \"$(pwd)/install/local_setup.bash\""
grep -qxF "$line_to_add" ~/.bashrc || echo "$line_to_add" >> ~/.bashrc
```
6.  **Запустить пример / основной функционал:**
launch-файл, который запускает только ноды из этого репозитория (подробнее см. 🧪 Использование)
```bash
ros2 launch completed_scripts_visualization show.launch.py
```
или
```bash
ros2 launch completed_scripts_visualization description.launch.py
```
Launch-файлы, которые запускают всю систему целиком смотри в [unitree_h1_meta_launch_ws](https://github.com/cyberbanana777/unitree_h1_meta_launch_ws?tab=readme-ov-file#%D0%B7%D0%B0%D0%BF%D1%83%D1%81%D0%BA-launch-%D1%84%D0%B0%D0%B9%D0%BB%D0%BE%D0%B2)

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## ⚙️ Предварительные требования

Что нужно для шагов "Быстрого Старта":
*   **Поддерживаемые версии ROS2:** Foxy
*   **Поддерживаемые платформы:** Ubuntu 20.04
*   **Ключевые ROS2 пакеты:** `rclpy`, `std_msgs`, `sensor_msgs`, `geometry_msgs`, `unitree_go`, `h1_info_library` `robot_state_publisher`, `imu_converter`, `joint_state_publisher_gui`, `rviz2`, `xarco`, `urdf`, `gazebo_ros_pkgs`, `tf2_ros`

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🧪 Использование

Как пользоваться пакетами после установки и сборки.

### **Запуск узлов:**
#### **h1_description**
##### Нода для публикации динамической TF-трансформации base_footprint => pelvis
```bash
ros2 run h1_description base_footprint_transform_node
```
#### **h1_move_joint_rviz**
##### Нода для визуализации целевых движений (можно без робота)
```bash
ros2 run h1_move_joint_rviz move_joint_rviz_without_real_robot_node
```
#####  Нода для визуализации реальных движений (нужно с роботом)
```bash
ros2 run h1_move_joint_rviz move_joint_rviz_with_real_robot_node
```

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

### **Запуск launch-файлов:**
#### 1. Основной launch-файл `show.launch.py`
##### Универсальный запуск с параметрами

```bash
ros2 launch completed_scripts_visualization show.launch.py
```
Доступные параметры запуска:
- `mode` - конфигурация робота: `with_hands` или `without_hands` (по умолчанию: `with_hands`)
- `launch_rviz` - включение RViz: `True` или `False` (по умолчанию: `False`)
- `launch_control_by_gui` - включение GUI управления суставами: `True` или `False` (по умолчанию: `False`)
- `robot` - тип робота: `real` (чтение состояния из /lowstate), `simulation` (чтение из /positions_to_unitree) или `empty` (для запуска `joint_state_publisher_gui`) (по умолчанию: `real`)

> 👉 Примечание:
> Симуляция робота в rviz может отражать его положение в пространстве по показаниям с датчиков IMU, подборбнее в [этом пунте](https://github.com/cyberbanana777/unitree_h1_visualization_ws?tab=readme-ov-file#-интерфейсы-топики-сервисы-действия-параметры) документации.

##### Примеры использования
Визуализация реального робота с руками:

```bash
ros2 launch completed_scripts_visualization show.launch.py mode:=with_hands launch_rviz:=True robot:=real
```

Визуализация положения, которое робот должен принять (значения берутся из топика `/positions_to_unitree`)  робота без рук с включенным RViz:

```bash
ros2 launch completed_scripts_visualization show.launch.py mode:=without_hands robot:=simulation launch_rviz:=True
```

Визуализация с GUI управлением суставами:

```bash
ros2 launch completed_scripts_visualization show.launch.py launch_control_by_gui:=True launch_rviz:=True robot:=empty
```

##### Запускаемые ноды (в зависимости от параметров):

**Всегда** запускается:

`h1_description` → `description.launch.py` (включает всю визуализацию робота)

При `robot:=real`:

`h1_move_joint_rviz` → `move_joint_rviz_with_real_robot_node`

При `robot:=simulation`:

`h1_move_joint_rviz` → `move_joint_rviz_without_real_robot_node`

#### 2. Базовый launch-файл `description.launch.py`
Может быть запущен отдельно для визуализации модели робота:

```bash
ros2 launch h1_description description.launch.py
```

Доступные параметры запуска:
- `mode` - конфигурация робота: `with_hands` или `without_hands` (по умолчанию: `with_hands`)
- `launch_rviz` - включение RViz: `True` или `False` (по умолчанию: `False`)
- `launch_control_by_gui` - включение GUI управления суставами: `True` или `False` (по умолчанию: `False`)

##### Примеры использования

Визуализация робота с руками и RViz:

```bash
ros2 launch h1_description description.launch.py mode:=with_hands launch_rviz:=True
```

Визуализация робота без рук с GUI управления суставами (только в визуализации):

```bash
ros2 launch h1_description description.launch.py mode:=without_hands launch_control_by_gui:=True launch_rviz:=True
```

##### Запускаемые ноды (в зависимости от параметров):

При `mode:=with_hands`:

`robot_state_publisher` → `robot_state_publisher` (загрузка URDF h1_with_hand.urdf)

При `mode:=without_hands`:

`robot_state_publisher` → `robot_state_publisher` (загрузка URDF h1.urdf)

При `launch_control_by_gui:=True`:

`joint_state_publisher_gui` → `joint_state_publisher_gui`

При `launch_rviz:=True`:

`rviz2` → `rviz2` (с конфигурацией check_joint.rviz)

**Всегда** запускается:

`h1_description` → `base_footprint_transform_node` (преобразование между `base_footprint` и `pelvis`)

#### Взаимосвязь launch-файлов:
```text
show.launch.py (основной)
    │
    └─── description.launch.py (базовая визуализация)
    │    ├── robot_state_publisher (URDF)
    │    ├── joint_state_publisher_gui (опционально)
    │    ├── rviz2 (опционально)
    │    └── base_footprint_transform_node
    │
    └─── move_joint_rviz_with_real_robot_node (для реального робота)
    или
    └─── move_joint_rviz_without_real_robot_node (для симуляции)
```

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 📚 Кастомные библиотеки
### **`h1_move_joint_rviz/rviz_util.py`**
Пакет `h1_move_joint_rviz` содержит не только ноды, но и мини-библиотеку, которая состоит из 1 файла - `rviz_util.py`.
В этом модуле содержится код, который используется в обоих нодах этого пакета. 
#### Объекты для взаимодействия
##### `rviz_util.py`
- **`LIMITS_URDF`** - словарь: ключ - индекс звена Unitree H1, значение - кортеж с пределами для этого звена в urdf-модели.
- **`START_POSITION`** - словарь: ключ - индекс звена Unitree H1, значение стартового положения для этого звена при отрисовке urdf-модели в rviz2.
- **`JOINTS_NAMES`** - список названий звеньев для urdf-модели
- **`map_range(value: float, in_min: float, in_max: float, out_min: float, out_max: float) -> float`** - функция, которая производит линейное преобразование значений из 1-ого диапазона в соответствующие им значения из 2-ого диапазона. Для корректной работы необходимы следующие аргументы:
	- `value` - значение, которое необходимо перевести в другую систему координат (из 1-ой во 2-ую);
	- `in_min` - наименьшее значение в 1-ом диапазоне
	- `in_max` - наибольшее значение в 1-ом диапазоне
	- `out_min` - наименьшее значение в 2-ом диапазоне
	- `out_max` - наибольшее значение в 2-ом диапазоне
	Возвращает значение из 2-го диапазона.

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 📡 Интерфейсы (топики, сервисы, действия, параметры)
Спецификация API пакетов.
### **Пакет 1: `h1_description`**
#### **Узел: `base_footprint_transform_node`**

**ВАЖНО!!!**
Данная нода показывает свои полные возможности при запущенной ноде-конвертере, которая переводит показания IMU из типа сообщения Unitree в стандартный тип сообщения ROS2 для IMU - `sensor_msgs/msg/Imu`. Данная нода является частью [другого нашего репозитория](https://github.com/cyberbanana777/unitree_h1_sensors_ws).
При отсутствии показаний с IMU нода всё равно будет работать, поворот систем координат `base_footprint` и `pelvis` будет отсутствовать.

- **Рабочие топики:**

| Тип услуги | Топик                     | Тип сообщения            | Описание                                                    |
| :--------- | :------------------------ | :----------------------- | :---------------------------------------------------------- |
| Публикация | `/tf`                     | `tf2_msgs/msg/TFMessage` | динамическая TF-трансформация `base_footprint` → `pelvis` ) |
| Подписка   | `/tf`                     | `tf2_msgs/msg/TFMessage` | Уже запущенные динамические TF-трансформации                |
| Подписка   | `/tf_static`              | `tf2_msgs/msg/TFMessage` | Уже запущенные статические TF-трансформации                 |
| Подписка   | `/sensors/imu/unitree_h1` | `sensor_msgs/msg/Imu`    | Показание IMU в корпусе робота                              |



- **Параметры:**

| Параметр              | Тип (знач. по умол.)                 | Описание                                                     |
| :-------------------- | :----------------------------------- | :----------------------------------------------------------- |
| `imu_topic`           | `string ('/sensors/imu/unitree_h1')` | Топик, из которого нода будет брать информацию `IMU`         |
| `frequency`           | `float (30.0)`                       | Частота публикации трансформации `base_footprint` → `pelvis` |

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

В launch-файле пакета `h1_description` присутствуют ноды, которые установлены в виде apt-пакетов. Вот их список (формат `package` → `executable`). Подробнее о их функционале читайте в интернете.
- `robot_state_publisher` → `robot_state_publisher`
- `rviz2` → `rviz2`
- `joint_state_publisher_gui` → ` joint_state_publisher_gui`

##### **Узел: `robot_state_publisher`**
- **Назначение:** Публикует TF-трансформации для всех звеньев робота на основе URDF-модели и состояний шарниров.

- **Рабочие топики:**

| Тип       | Топик         | Тип сообщения                  | Описание                                                                 |
| :-------- | :------------ | :----------------------------- | :----------------------------------------------------------------------- |
| Подписка  | `/joint_states` | `sensor_msgs/msg/JointState`   | Состояния шарниров робота (позиции, скорости, усилия)                   |
| Публикация | `/tf`         | `tf2_msgs/msg/TFMessage`       | Динамические TF-трансформации (например, для подвижных звеньев)         |
| Публикация | `/tf_static`  | `tf2_msgs/msg/TFMessage`       | Статические TF-трансформации (фиксированные связи из URDF)              |

- **Параметры:**

| Параметр             | Тип (знач. по умол.)      | Описание                                                                 |
| :------------------- | :------------------------ | :----------------------------------------------------------------------- |
| `robot_description`  | `string`                  | **Обязательный параметр.** Содержимое URDF-модели робота в формате XML.  |
| `publish_frequency`  | `double (50.0)`           | Частота публикации TF-трансформаций (в Гц).                              |
| `frame_prefix`       | `string ('')`             | Префикс для всех TF-фреймов (обычно используется в мультироботных сценариях). |

##### **Узел: `rviz2`**
- **Назначение:** Визуализация данных ROS2 (TF-трансформации, сенсоры, модели и т.д.) в интерактивном 3D-окне.

- **Рабочие топики:**

| Тип       | Топик                     | Тип сообщения                  | Описание                                                                 |
| :-------- | :------------------------ | :----------------------------- | :----------------------------------------------------------------------- |
| Подписка  | `/tf`                     | `tf2_msgs/msg/TFMessage`       | Динамические TF-трансформации для отображения иерархии фреймов.          |
| Подписка  | `/tf_static`              | `tf2_msgs/msg/TFMessage`       | Статические TF-трансформации.                                            |
| Подписка  | `~/markers`               | `visualization_msgs/msg/Marker` | Визуальные маркеры (например, стрелки, тексты, фигуры).                 |
| Подписка  | Произвольные топики       | Зависит от типа данных         | Подписки на топики для отображения данных (например, облака точек, изображения). |

- **Параметры:**

| Параметр          | Тип (знач. по умол.)        | Описание                                                                 |
| :---------------- | :-------------------------- | :----------------------------------------------------------------------- |
| `display_config`  | `string`                    | Путь к файлу конфигурации RViz2 (сохраняет настройки интерфейса).        |

##### **Узел: `joint_state_publisher_gui`**
- **Назначение:** GUI-инструмент для ручной публикации состояний шарниров (через слайдеры).

- **Рабочие топики:**

| Тип       | Топик         | Тип сообщения                | Описание                                                                 |
| :-------- | :------------ | :--------------------------- | :----------------------------------------------------------------------- |
| Публикация | `/joint_states` | `sensor_msgs/msg/JointState` | Публикует состояния шарниров (позиции) на основе ввода пользователя.    |

- **Параметры:**

| Параметр             | Тип (знач. по умол.) | Описание                                                                 |
| :------------------- | :------------------- | :----------------------------------------------------------------------- |
| `robot_description`  | `string`             | **Обязательный параметр.** URDF-модель робота для определения шарниров.  |
| `rate`               | `float (10.0)`       | Частота публикации сообщений `JointState`.                               |

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

### **Пакет 2: `h1_move_joint_rviz`**
#### **Узел: `move_joint_rviz_without_real_robot_node`**
- **Рабочие топики:**

| Тип услуги | Топик                   | Тип сообщения                 | Описание                                                                                |
| :--------- | :---------------------- | :---------------------------- | :-------------------------------------------------------------------------------------- |
| Публикация | `/joint_states`         | `sensors_msgs/msg/JointState` | Угловые положения суставов Unitree H1 для визуализации в rviz2                        |
| Подписка   | `/positions_to_unitree` | `std_msgs/msg/String`         | Целевые положения суставов Unitree H1 в допустимом диапазоне для Unitree H1 |

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

#### **Узел: `move_joint_rviz_without_real_robot_node`**
- **Рабочие топики:**

| Тип услуги   | Топик            | Тип сообщения                    | Описание                                                               |
| :----------- | :--------------- | :------------------------------- | :--------------------------------------------------------------------- |
| Публикация   | `/joint_states`  | `sensors_msgs/msg/JointState`    | Угловые положения суставов Unitree H1 для визуализации в rviz2       |
| Подписка     | `/wrist/states`  | `unitree_go/msg/MotorStates`<br> | Состояния моторов, которые расположены в кистях робота `Unitree H1`    |
| Подписка<br> | `/inspire/state` | `unitree_go/msg/MotorStates`     | Степень разжатия пальцев `Inspire Hands` в условных единицах от 0 до 1 |
| Подписка<br> | `/lowstate`      | `unitree_go/msg/LowState`        | Состояния суставов робота `Unitree H1`                                 |

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🗺️ Архитектура 
> Данный раздел находится в разработке. Графы появятся позже... 😉
Здесь представлены скрины из rqt_graph при различных запущенных сценариях.
### `h1_description`
#### `.py` 

#### `.py`

#### `.py`

#### `.py`

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

### `completed_scripts_visualization`
##### `.py`

##### `.py`

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 📜 Лицензия
Copyright (c) 2025 Алиса Зенина и Александр Грачев РТУ МИРЭА (Россия)

Данное программное обеспечение распространяется под [лицензией MIT](LICENSE).  
Разрешается свободное использование, копирование, модификация и распространение при условии сохранения уведомления об авторских правах и текста лицензии.

## 🙏 Благодарности
- Часть кода (пакет `h1_description`) основана на [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros?tab=readme-ov-file) ([BSD 3-Clause](h1_description/LICENSE-ORIGINAL)).  
**Благодарим всех, кто косвенно участвовал в разработке.**

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 💡 Предложения и корректировки
Если Вы нашли, ошибку, неточность, у Вас есть предложения по улучшению или вопросы, то напишите в телеграмм [сюда](https://t.me/Alex_19846) (Александр) или [сюда](https://t.me/Kika_01) (Алиса).

<p align="right" style="margin-top: 20px;"><a href="#-оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>
