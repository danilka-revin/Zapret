# Лидар через ROS 2: установка, запуск, диагностика

Документ для робота: как лидар отдаёт скан **пакетом ROS 2**, а пульт (8080) и
бортовой экран (8090) читают его из топика `/scan`.

Коротко:

```bash
./scripts/ros2_setup.sh --install --drivers   # ROS 2 + пакеты драйверов + сборка ws
./scripts/ros2_lidar.sh                       # запустить драйвер лидара
./scripts/ros2_lidar.sh --status              # топик /scan, частота, состояние драйвера
./scripts/stop.sh && ./scripts/start_robot.sh # перезапустить оба экрана
```

## Зачем это вместо чтения USB-порта приложением

Раньше приложение само открывало USB-порт лидара и подбирало протокол, скорость
и линию включения мотора. На живом роботе это давало три проблемы:

1. **Порт один, а читателей несколько.** Пульт и бортовой экран, источник данных
   шасси и сам лидар по очереди занимали один и тот же `/dev/ttyUSB*`; пока шёл
   подбор, скан пропадал, а в журнале появлялось «порт занят другой программой».
2. **LDS-01 через плату OpenCR** (заводская сборка TurtleBot3 Burger) вообще не
   виден как USB-устройство лидара: скан отдаёт плата робота.
3. **`rclpy` не виден из `.venv`.** ROS 2 ставится системно в `/opt/ros`, а
   приложение работает интерпретатором из `.venv` — без подготовки путей импорт
   `rclpy` падал, и ветка «взять скан из ROS 2» не включалась никогда, хотя
   `ros2 topic echo /scan` в терминале работал.

Теперь порт занимает **драйвер производителя** (пакет ROS 2), а приложение только
подписывается на топик: спорить за порт больше некому. Путь и переменные
окружения для `rclpy` добавляет `robot_control/ros_env.py`.

## Что стоит на роботе

| Лидар | Пакет ROS 2 | Узел | Откуда скан |
| --- | --- | --- | --- |
| LDS-01/LDS-02 (TurtleBot3 Burger, HLS-LFCD2) | `hls_lfcd_lds_driver` | `hlds_laser_publisher` | USB-переходник лидара, 230400 |
| RPLIDAR A1/A2M8 | `rplidar_ros` | `rplidar_composition` / `rplidar_node` | 115200 |
| RPLIDAR A2M7/A2M12/A3/S1 | `rplidar_ros` | то же | 256000 |
| RPLIDAR C1 | `rplidar_ros` | то же | 460800 |
| RPLIDAR (альтернатива) | `sllidar_ros2` | `sllidar_node` | по модели |
| LD06/LD19 | `ldlidar_stl_ros2` | `ldlidar_stl_ros2_node` | 230400 |
| STL-27L | `ldlidar_stl_ros2` | `ldlidar_stl_ros2_node` | 921600 |
| LDS-01 через плату OpenCR | `turtlebot3_node` | `turtlebot3_ros` | `/dev/ttyACM0`, `param/burger.yaml` |

Таблицу знает супервизор `zapret_lidar`: он и решает, какой пакет запускать.
`ldlidar_stl_ros2` в apt не публикуется — `ros2_setup.sh --drivers` клонирует и
собирает его из исходников.

## Установка

```bash
./scripts/ros2_setup.sh                 # найти ROS 2, собрать ros2_ws, включить RC_LIDAR_SOURCE=ros
./scripts/ros2_setup.sh --check --json  # что найдено (дистрибутив, rclpy, пакеты)
./scripts/ros2_setup.sh --install       # поставить ROS 2 (apt), если его нет
./scripts/ros2_setup.sh --drivers       # поставить пакеты драйверов лидаров
./scripts/ros2_setup.sh --ldlidar       # склонировать и собрать ldlidar_stl_ros2
./scripts/ros2_setup.sh --service       # служба robot-lidar (лидар поднимается до приложений)
```

`--install` подключает официальный репозиторий ROS 2 и ставит `ros-<дистрибутив>-ros-base`,
`ros-dev-tools`, `colcon` и `python3-serial`. Дистрибутив угадывается по версии
Ubuntu (22.04 → `humble`, 24.04 → `jazzy`), можно задать вручную: `--distro humble`.

`--service` ставит юнит `robot-lidar.service` с `Before=robot-control.service`:
драйвер лидара поднимается раньше приложений, и они сразу читают `/scan`.
`./scripts/autostart.sh off` гасит и эту службу.

## Запуск и проверка

```bash
./scripts/ros2_lidar.sh                        # модель и порт — из .env
./scripts/ros2_lidar.sh --model lds01 --port /dev/ttyUSB0
./scripts/ros2_lidar.sh --status               # список топиков, состояние, частота /scan
./scripts/ros2_lidar.sh --print                # показать команду, ничего не запуская
```

Вручную, «как в документации производителя»:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
cd ros2_ws && colcon build --symlink-install && source install/setup.bash
ros2 launch zapret_lidar lidar.launch.py model:=auto
```

Диагностика без ROS 2 (план запуска считается чистой стандартной библиотекой):

```bash
python3 ros2_ws/src/zapret_lidar/zapret_lidar/supervisor.py --print-plan
python3 ros2_ws/src/zapret_lidar/zapret_lidar/supervisor.py --list-drivers
```

## Как супервизор выбирает драйвер

1. Собирает план: какие пакеты драйверов реально установлены (ament-индекс),
   какие есть порты (`/dev/serial/by-id/*lidar*`, `/dev/ttyUSB*`, `/dev/ttyACM*`;
   порты мастера руления, шасси и BMS исключены).
2. При `model:=auto` быстро проверяет порты и ставит вперёд драйвер того лидара,
   чьи кадры видны: LDS-01 (`0xFA`, 42 байта), LD06/LD19 (`0x54 0x2C`), RPLIDAR
   (ответ `A5 5A` на команду SCAN и ровный виток точек).
3. Запускает драйвер напрямую из `lib/<package>/` (или `ros2 run`), ждёт
   `sensor_msgs/LaserScan` до `probe_timeout` (8 с) и следит за свежестью:
   процесс упал или точки пропали — перезапуск, потом следующий драйвер из плана.
4. Публикует состояние в `/zapret/lidar/status` (JSON). Оно попадает в
   `GET /api/lidar` → `ros.supervisor` и в панель «Лидар» обоих экранов:
   драйвер, порт, точки, Гц, что уже пробовали и чего не хватает.

Флаги и параметры — в `ros2_ws/src/zapret_lidar/config/lidar.yaml`
(`model`, `port`, `baud`, `scan_topic`, `status_topic`, `probe_timeout`,
`scan_timeout`, `autostart`, `autoprobe`).

## Настройки `.env`

| Переменная | Значение | Смысл |
| --- | --- | --- |
| `RC_LIDAR_SOURCE` | `auto` | `auto` — ROS 2, если он есть, иначе USB; `ros` — только ROS 2; `serial` — только USB (приложением) |
| `RC_LIDAR_ROS_TOPIC` | `/scan` | топик скана, который слушают оба экрана |
| `RC_LIDAR_PROTOCOL` | `auto` | в ROS 2-режиме это модель лидара: `lds01`, `rplidar`, `ld06`, `turtlebot3`… — супервизор запускает соответствующий пакет |
| `RC_LIDAR_PORT` | пусто | порт для драйвера; пусто — автопоиск |
| `RC_LIDAR_BAUD` | пусто | скорость порта; пусто — по умолчанию драйвера |
| `RC_LIDAR_ROS_STATUS_TOPIC` | `/zapret/lidar/status` | топик состояния супервизора |
| `RC_ROS_PREFIX` | `/opt/ros/<дистрибутив>` | где искать ROS 2, если он не в `/opt/ros` |

Все они доступны и из панели «Лидар» (веб-пульт — после входа, бортовой экран —
без PIN): в списке «Откуда берётся скан» есть `auto`, `ros` и `serial`.

## Если скана нет

| Что видно | Что делать |
| --- | --- |
| `ROS 2 не установлен (нет /opt/ros/…)` | `./scripts/ros2_setup.sh --install --drivers`, затем перезапустить приложения |
| `rclpy не импортируется из /opt/ros/…` | поставить `ros-<дистрибутив>-rclpy` (или полный `ros-base`) и проверить `./scripts/ros2_setup.sh --check` |
| `ROS 2 есть, но /scan никто не публикует — драйвер лидара не запущен` | `./scripts/ros2_lidar.sh` (или включить службу: `./scripts/ros2_setup.sh --service`) |
| `нет ROS 2-пакетов драйверов лидара: …` | `./scripts/ros2_setup.sh --drivers` (для LD06/LD19 — ещё `--ldlidar`) |
| `ни один драйвер не отдал /scan (rplidar@/dev/ttyUSB0 — нет /scan за 8 с)` | проверить порт и права: `./scripts/lidar_check.py --ros`, `sudo usermod -aG dialout $USER`, перезайти |
| `драйвер встал: нет точек 3.0 с` | драйвер перезапустится сам; если повторяется — смотрите `journalctl -u robot-lidar -f` |
| `на портах … не видно ни одного известного лидара` | лидар подключён к плате робота (OpenCR): задайте `--model opencr` или `RC_LIDAR_PROTOCOL=turtlebot3` |
| скан идёт, но хочется старый путь | `RC_LIDAR_SOURCE=serial` (только USB) — прежнее чтение порта приложением целиком сохранено |

Проверка «глазами ROS 2»:

```bash
source /opt/ros/$ROS_DISTRO/setup.bash
ros2 topic list | grep -E 'scan|zapret'
ros2 topic hz /scan
ros2 topic echo --once /zapret/lidar/status
```

Проверка со стороны приложения:

```bash
.venv/bin/python scripts/lidar_check.py --ros         # дистрибутив, rclpy, пакеты драйверов
.venv/bin/python scripts/lidar_web_check.py           # дошёл ли скан до обоих экранов
curl -s http://127.0.0.1:8080/api/lidar | python3 -m json.tool | head -40
```

## Что осталось на прежнем пути

Чтение USB-порта самим приложением (`RC_LIDAR_SOURCE=serial` и `auto` без ROS 2)
никуда не делось: на стенде без ROS 2 и при отладке оно работает как раньше —
перебор портов, протоколов и скоростей, межпроцессный замок порта, обмен сканом
между пультом и бортом по HTTP (`peer`). ROS 2-путь просто стоит первым, когда
ROS 2 в системе есть.
