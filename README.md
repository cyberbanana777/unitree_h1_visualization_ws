# unitree_h1_visualization_ws

В данном репозитории лежат ROS2-пакеты, необходимые для визуализации движений робота Unitree H1.
Пакты которые задают даижения робота см. [meta_launch_ws](https://github.com/cyberbanana777/unitree_h1_meta_launch_ws?tab=readme-ov-file#-какие-репозитории-установить).

## Оглавление

1. [📦 Содержание репозитория](#-содержание-репозитория)
2. [🚀 Быстрый старт](#-быстрый-старт)
3. [⚙️ Предварительные требования](#️-предварительные-требования)
4. [🧪 Использование](#-использование)
   - 4.1 [Запуск узлов](#запуск-узлов)
   - 4.2 [Запуск launch-файлов](#запуск-launch-файлов)
5. [📚 Кастомные библиотеки](#-кастомные-библиотеки)
   - 5.1 [`rviz_util.py`](#rviz_utilpy)
6. [📡 Интерфейсы](#-интерфейсы)
   - 6.1 [Пакет `h1_description`](#пакет-h1_description)
   - 6.2 [Пакет `h1_move_joint_rviz`](#пакет-h1_move_joint_rviz)
7. [🗺️ Архитектура](#️-архитектура)
   - 7.1 [Пакет `h1_description`](#пакет-h1_description-1)
   - 7.2 [Пакет `completed_scripts_visualization`](#пакет-completed_scripts_visualization)
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

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🚀 Быстрый старт
Пошаговая инструкция для **быстрого** запуска основной функциональности. Предполагаем, что ROS2 (Foxy) уже установлен.
    
‼️**Репозиторий нужно устанавливать на локальной машине, а не удаленном компьютере робота, для быстрой и корректной работы графическго интерфейса.**

1.  **Клонировать репозиторий** в `src` вашего workspace:
```bash
mkdir -p unitree_h1_visualization_ws/src
cd unitree_h1_visualization_ws/src
git clone https://github.com/cyberbanana777/unitree_h1_visualization.git .
my_pwd=$(pwd)
```
2. **Установить проприетарные зависимости** (по ссылкам инструкции по установке от производителя): 
-   [unitree_h1_control_ws](https://github.com/cyberbanana777/unitree_h1_control_ws) - Репозиторий, котоый нужно установить, в нем находится библиотека, содержащая наиболее часто используемые функции, словари и тд при работе с Unitree H1. Инструкция по установке есть внутри репозитория и также есть bash-скрипт, котоый полоностью устанавливает репозиторий и все зависммости и библиотеки. Находится он [здесь](https://github.com/cyberbanana777/unitree_h1_meta_launch_ws?tab=readme-ov-file#соло-скрипты). Подробнее читайте в инструкции к репозиториям.
- [unitree_ros2](https://github.com/unitreerobotics/unitree_ros2). Если вы уже установили данную зависимость при установке другого репозитория, повторно выполнять не нужно. По ссылке выполняете всё, что написано до `Connect to Unitree robot`. После всех действий нужно выполнить команду:
```bash
echo "source ~/unitree_ros2/cyclonedds_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc 
```
3. **Установить pip-зависимости:**
Для простоты мы сделали bash-скрипт, который автоматизирует этот процесс. Его необходимо запустить. Для этого выполним команды:
```bash
cd $my_pwd
chmod +x install_dependensies.bash
./install_dependensies.bash
```
4.  **Собрать workspace:**
```bash
cd $my_pwd/..
colcon build # --symlink-install позволяет не colcon'ить область после изменений кода пакетов, что бы изменения были видны системе.
source install/setup.bash  # Или setup.zsh - в зависимости от вашего интерпретатора командной строки
```
5. Добавить `source` в `~/.bashrc`: 
Зачем? - Чтобы не делать при каждом перезапуске терминала `source install/setup.bash`. Выполните команды ниже и это изменит код `~/.bashrc` - скрипта, который выполняется при запуске нового терминала. Для каждой ws делается отдельно. 
```bash
line_to_add="source \"$(pwd)/install/setup.bash\""
grep -qxF "$line_to_add" ~/.bashrc || echo "$line_to_add" >> ~/.bashrc
```
6.  **Запустить пример / основной функционал:**
launch-файл, который запускает только ноды из этого репозитория (подробнее см. 🧪 Использование)
```bash
ros2 launch completed_scripts_visualization show_and_move_h1_with_real_robot_launch.py
```
или
```bash
ros2 launch completed_scripts_visualization show_and_move_h1_without_real_robot_launch.py
```
Launch-файлы, которые запускают всю систему целиком смотри в [unitree_h1_meta_launch_ws](https://github.com/cyberbanana777/unitree_h1_meta_launch_ws?tab=readme-ov-file#%D0%B7%D0%B0%D0%BF%D1%83%D1%81%D0%BA-launch-%D1%84%D0%B0%D0%B9%D0%BB%D0%BE%D0%B2)

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## ⚙️ Предварительные требования

Что нужно для шагов "Быстрого Старта":
*   **Поддерживаемые версии ROS2:** Foxy
*   **Поддерживаемые платформы:** Ubuntu 20.04
*   **Ключевые ROS2 пакеты:** `rclpy`, `std_msgs`, `sensor_msgs`, `unitree_go`, `h1_info_library` `robot_state_publisher`, `joint_state_publisher`, `joint_state_publisher_gui`, `rviz2`, `xarco`, `urdf`, `gazebo_ros_pkgs`, `tf2_ros`

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🧪 Использование

Как пользоваться пакетами после установки и сборки.

### **Запуск узлов:**
#### **h1_description**
Узлы отсутствуют
#### **h1_move_joint_rviz**#
##### Нода для визуализации целевых движений (можно без робота)
```bash
ros2 run h1_move_joint_rviz move_joint_rviz_without_real_robot_node
```
#####  Нода для визуализации реальных движений (нужно с роботом)
```bash
ros2 run h1_move_joint_rviz move_joint_rviz_with_real_robot_node
```

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

### **Запуск Launch файлов:**
#### Пакет `h1_description`
##### Запуск Rviz2 (визуализатор) с настроенной конфигурацией
```bash
ros2 launch h1_description 
rviz_with_config_launch.py
```
**Запускаемые ноды** (в формате `пакет -> зависимость`, `пакет => launch-файл`):
- `rviz2 -> rviz2
##### Запуск публикации описания робота и TF-преобразования `base_footprint -> pelvis`
В контексте описания робота, фрейм `pelvis` является аналогом стандартного `base_link`.
```bash
ros2 launch h1_description 
robot_description_and_tfs_launch.py
```
**Запускаемые ноды** (в формате `пакет -> зависимость`, `пакет => launch-файл`):
- `robot_state_publisher -> robot_state_publisher`
- `tf2_ros -> static_transform_publisher`
##### Запуск отображения робота в Rviz (визуализаторе)
```bash
ros2 launch h1_description 
display_without_control_launch.py
```
**Запускаемые ноды** (в формате `пакет -> зависимость`, `пакет => launch-файл`):
- `h1_description => rviz_with_config_launch.py`
- `h1_description => robot_description_and_tfs_launch.py`
##### Запуск отображения робота в Rviz (визуализаторе) c возможностью задать положения суставов в GUI-интерфейсе
##### Запуск отображения робота в Rviz (визуализаторе)
```bash
ros2 launch h1_description 
display_with_control_launch.py
```
**Запускаемые ноды** (в формате `пакет -> зависимость`, `пакет => launch-файл`):
- `h1_description => rviz_with_config_launch.py`
- `h1_description => robot_description_and_tfs_launch.py`
- `joint_state_publisher_gui -> joint_state_publisher_gui`

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

#### Пакет `completed_scripts_visualization`
##### Запуск визуализации с движений реального робота
```bash
ros2 launch completed_scripts_visualization show_and_move_h1_with_real_robot_launch.py
```
**Запускаемые ноды** (в формате `пакет -> зависимость`, `пакет => launch-файл`):
- `h1_description => display_without_control_launch.py`
- `h1_move_joint_rviz -> move_joint_rviz_with_real_robot_node`
##### Запуск визуализации с целевых позиций робота
```bash
ros2 launch completed_scripts_visualization show_and_move_h1_without_real_robot_launch.py
```
**Запускаемые ноды** (в формате `пакет -> зависимость`, `пакет => launch-файл`):
- `h1_description => display_without_control_launch.py`
- `h1_move_joint_rviz -> move_joint_rviz_without_real_robot_node`

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## Кастомные библиотеки
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
	- `in_max` - наибольшее значение в 1-ом дапазоне
	- `out_min` - наименьшее значение в 2-ом диапазоне
	- `out_max` - наибольшее значение в 2-ом диапазоне
	Возвращаеет значение из 2-го диапазона.

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 📡 Интерфейс (топики, сервисы, действия, параметры)
Спецификация API пакетов.
### **Пакет 1: `h1_descroption`**
Узлы отсутствуют.
Но в launch-файлах присутствуют ноды, которые установлены через apt-пакеты. Вот их список (формат `package -> executable`). Подробнее о их функционале читайте в интернете.
- `robot_state_publisher -> robot_state_publisher`
- `rviz2 -> rviz2`
- `tf2_ros -> static_transform_publisher`
- `joint_state_publisher_gui -> joint_state_publisher_gui`

| Тип услуги | Топик            | Тип сообщения         | Описание                                                                                           |
| :--------- | :--------------- | :-------------------- | :------------------------------------------------------------------------------------------------- |
> Возможно, здесь появится инфа про эти ноды... 😉

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

### **Пакет 2: `h1_move_joint_rviz`**
#### **Узел: `move_joint_rviz_without_real_robot_node`**
- **Рабочие топики:**

| Тип услуги | Топик                   | Тип сообщения                 | Описание                                                                                |
| :--------- | :---------------------- | :---------------------------- | :-------------------------------------------------------------------------------------- |
| Публикация | `/joint_states`         | `sensors_msgs/msg/JointState` | Угловые положения суставов Unitree H1 для визуализации в rviz2                        |
| Подписка   | `/positions_to_unitree` | `std_msgs/msg/String`         | Целевые положения суставов Unitree H1 в допустимом диапазоне для Unitree H1 |

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>
#### **Узел: `move_joint_rviz_without_real_robot_node`**
- **Рабочие топики:**

| Тип услуги   | Топик            | Тип сообщения                    | Описание                                                               |
| :----------- | :--------------- | :------------------------------- | :--------------------------------------------------------------------- |
| Публикация   | `/joint_states`  | `sensors_msgs/msg/JointState`    | Угловые положения суставов Unitree H1 для визуализации в rviz2       |
| Подписка     | `/wrist/states`  | `unitree_go/msg/MotorStates`<br> | Состояния моторов, которые расположены в кистях робота `Unitree H1`    |
| Подписка<br> | `/inspire/state` | `unitree_go/msg/MotorStates`     | Степень разжатия пальцев `Inspire Hands` в условных единицах от 0 до 1 |
| Подписка<br> | `/lowstate`      | `unitree_go/msg/LowState`        | Состояния суставов робота `Unitree H1`                                 |

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## 🗺️ Архитектура 
Здесь представлены скрины из rqt_graph при различных запущенных сценариях.
### `h1_description`
#### `rviz_with_config_launch.py` 
> Здесь будет картинка
#### `robot_description_and_tfs_launch.py`
> Здесь будет картинка
#### `display_with_control_launch.py`
> Здесь будет картинка
#### `display_without_control_launch.py`
> Здесь будет картинка

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

### `completed_scripts_visualization`
##### `show_and_move_h1_with_real_robot_launch.py`
> Здесь будет картинка
##### `show_and_move_h1_without_real_robot_launch.py`
> Здесь будет картинка
<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## Лицензия
Copyright (c) 2025 Алиса Зенина и Александр Грачев РТУ МИРЭА (Россия)

Данное программное обеспечение распространяется под [лицензией MIT](LICENSE).  
Разрешается свободное использование, копирование, модификация и распространение при условии сохранения уведомления об авторских правах и текста лицензии.

## Благодарности
- Часть кода (пакет `h1_description`) основана на [Unitree Robotics](https://github.com/unitreerobotics/unitree_ros?tab=readme-ov-file) ([BSD 3-Clause](h1_description/LICENSE-ORIGINAL)).  
**Благодарим всех, кто косвенно участвовал в разработке.**
<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>

## Предложения и корректировки
Если Вы нашли, ошибку, неточность, у Вас есть предложения по улучшению или вопросы, то напишите в телеграмм [сюда](https://t.me/Alex_19846) (Александр) или [сюда](https://t.me/Kika_01) (Алиса).

<p align="right" style="margin-top: 20px;"><a href="#оглавление" style="text-decoration: none;">🔝 Вернуться к оглавлению</a></p>
