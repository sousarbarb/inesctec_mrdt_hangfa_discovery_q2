# inesctec_mrdt_hangfa_discovery_q2

Repository contains all the documentation associated with the modifications
made by [INESC TEC](https://www.inesctec.pt/en/) on the
[Hangfa Discovery Q2](https://www.hangfa-europe.com/en/omni-robot/discovery)
mobile platform in order to be compatible with multimodal perception. These
modifications also enable the platform to be integrated in the
[Robot Operating System (ROS)](https://ros.org/), facilitating its usage for
research topics such as perception, localisation and mapping, multi-robot
coordination (when more than one platform is available to the user), Artificial
Intelligence (AI) applied on autonomous mobile robotics, among other topics.

## Setup

### Mobile Platform

Check the
[GitHub Pages website](https://sousarbarb.github.io/inesctec_mrdt_hangfa_discovery_q2/)
for more information on the multimodal modifications made to the original
[Hangfa Discovery Q2](https://www.hangfa-europe.com/en/omni-robot/discovery)
mobile platform.

### Firmware

See [README.md](src/firmware/inesctec_mrdt_discovery_q2_fw/README.md) in the
firmware source folder
([src/firmware/inesctec_mrdt_discovery_q2_fw](src/firmware/inesctec_mrdt_discovery_q2_fw/)).

### Robot Operating System (ROS)

- Drivers:
  - [inesctec_mrdt_drivers_interfaces](src/drivers/inesctec_mrdt_drivers_interfaces)
  - [inesctec_mrdt_hangfa_discovery_q2_driver](src/drivers/inesctec_mrdt_hangfa_discovery_q2_driver)
  - [inesctec_mrdt_omnijoy_driver](src/drivers/inesctec_mrdt_omnijoy_driver)
  - [inesctec_mrdt_serial_channels](src/drivers/inesctec_mrdt_serial_channels)
  - [inesctec_mrdt_serial_port](src/drivers/inesctec_mrdt_serial_port)
  - [livox_ros_driver2](src/drivers/livox_ros_driver2)
    _(may be removed from repo to allow further compatibility with ROS versions)_
  - [rslidar_msg](src/drivers/rslidar_msg)
    _(may be removed from repo to allow further compatibility with ROS versions)_
  - [rslidar_sdk](src/drivers/rslidar_sdk)
    _(may be removed from repo to allow further compatibility with ROS versions)_
- Localization:
  [inesctec_mrdt_localization_odom](src/localization/inesctec_mrdt_localization_odom)
- Launch system:
  [inesctec_mrdt_hangfa_discovery_q2_nav_conf](src/inesctec_mrdt_hangfa_discovery_q2_nav_conf)

**ROS supported versions**
- ROS 1
  - [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
  - [ROS Noetic](https://wiki.ros.org/noetic)
- ROS 2
  - [Ubuntu 20.04.6 LTS](https://releases.ubuntu.com/focal/)
  - [ROS 2 Foxy](https://docs.ros.org/en/foxy/)

_Note:_ remove LiDAR specific packages if you want to compile the original
packages developed in the scope of this work on more recent ROS versions.

**Dependencies**
```sh
sudo apt-get install -y libpcap-dev
sudo apt-get install -y ros-$ROS_DISTRO-joy      \
                        ros-$ROS_DISTRO-pcl-ros  \
                        ros-$ROS_DISTRO-libpointmatcher
```

```sh
# Livox-SDK2
git clone https://github.com/Livox-SDK/Livox-SDK2.git
cd Livox-SDK2/
mkdir build
cd build/
cmake .. && make -j
sudo make install
```

```sh
# yaml-cpp
git clone https://github.com/jbeder/yaml-cpp.git
cd yaml-cpp/
mkdir build
cd build/
cmake .. -DYAML_BUILD_SHARED_LIBS=ON && make -j
sudo make install
```

## Usage

### Repository
```sh
git clone git@github.com:sousarbarb/inesctec_mrdt_hangfa_discovery_q2.git --init --recursive
```

### Build
**ROS 1**
```sh
source /opt/ros/$ROS_DISTRO/setup.bash

cd inesctec_mrdt_hangfa_discovery_q2/src
catkin_init_workspace

cd ..
catkin_make --force-cmake --cmake-args -DCMAKE_BUILD_TYPE=Release

source devel/setup.bash
```

**ROS 2**
```sh
source /opt/ros/$ROS_DISTRO/setup.bash

cd inesctec_mrdt_hangfa_discovery_q2
colcon build --cmake-args -DCMAKE_BUILD_TYPE=Release --event-handlers status+ console_direct+ console_start_end+

source install/setup.bash
```

### Launch

**ROS 1**
```sh
# Robot id
export ROBOT_ID=<id>                # (default: unnamed_robot)
# Configuration
export ROBOT_CONF=<configuration>   # (default: basic)

roslaunch inesctec_mrdt_hangfa_discovery_q2_nav_conf wake_up_dumb_q2.launch
```

**ROS 2**
```sh
# Robot id
export ROBOT_ID=<id>                # (default: unnamed_robot)
# Configuration
export ROBOT_CONF=<configuration>   # (default: basic)

ros2 launch inesctec_mrdt_hangfa_discovery_q2_nav_conf wake_up_dumb_q2.launch.xml
```

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the following contributors:

- Ricardo B. Sousa
  ([rbs@fe.up.pt](mailto:rbs@fe.up.pt))
  _(Corresponding Author)_
  ([github](https://github.com/sousarbarb/),
  [gitlab](https://gitlab.com/sousarbarb/),
  [gitlab](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [orcid](https://orcid.org/0000-0003-4537-5095),
  [google-scholar](https://scholar.google.pt/citations?user=Bz2FMqYAAAAJ),
  [linkedin](https://www.linkedin.com/in/sousa-ricardob/),
  [youtube](https://www.youtube.com/channel/UCXTR8mMlG0VOC_06PKg5KBQ))
- Héber Miguel Sobreira
  ([heber.m.sobreira@inesctec.pt](mailto:heber.m.sobreira@inesctec.pt))
  ([github](https://github.com/HeberSobreira),
  [gitlab](https://gitlab.inesctec.pt/heber.m.sobreira/),
  [orcid](https://orcid.org/0000-0002-8055-1093),
  [google-scholar](https://scholar.google.pt/citations?user=iNhGcpsAAAAJ))
- João G. Martins
  ([joao.g.martins@inesctec.pt](mailto:joao.g.martins@inesctec.pt))
  ([github](https://github.com/Joao-G-Martins),
  [orcid](https://orcid.org/0000-0002-6567-4802))
- Paulo G. Costa
  ([paco@fe.up.pt](mailto:paco@fe.up.pt))
  ([github](https://github.com/P33a),
  [orcid](https://orcid.org/0000-0002-4846-271X),
  [google-scholar](https://scholar.google.pt/citations?user=7Iz8fKcAAAAJ))
- Manuel F. Silva
  ([mss@isep.ipp.pt](mailto:mss@isep.ipp.pt))
  ([orcid](https://orcid.org/0000-0002-0593-2865),
  [google-scholar](https://scholar.google.pt/citations?user=2EFVZ-AAAAAJ))
- António Paulo Moreira
  ([amoreira@fe.up.pt](mailto:amoreira@fe.up.pt))
  ([orcid](https://orcid.org/0000-0001-8573-3147),
  [google-scholar](https://scholar.google.pt/citations?user=eL0gHLoAAAAJ))

## Institutions

- [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
- [Faculty of Engineering, University of Porto (FEUP)](https://sigarra.up.pt/feup/en/)

## Acknowledgements

- [5dpo Robotics Team](https://5dpo.github.io/)
- [Amorins & Silva](https://amorinsesilva.pt/)
- [Hangfa Robotics Europe](https://www.hangfa-europe.com/)
- [LattePanda Team](https://www.lattepanda.com/)

## Funding

This work is co-financed by Component 5 - Capitalization and Business
Innovation, integrated in the Resilience Dimension of the Recovery and
Resilience Plan within the scope of the Recovery and Resilience
Mechanism (MRR) of the European Union (EU), framed in the Next Generation EU,
for the period 2021-2026, within project GreenAuto, with reference 54.

**GreenAuto: Green innovation for the Automotive Industry**

- **Operation Code:** 02/C05-i01.02/2022.PC644867037-00000013
- **Beneficiary:** Peugeot Citröen Automóveis Portugal, S.A.
- **Work Package:** WP10 - Automated logistics for the automotive industry
- **Product, Processes, or Services (PPS):**
  PPS18 - 3D navigation system for mobile robotic equipment
- **Consortium Partners:**
    - [Flowbotic Mobile Systems, Lda](https://www.flowbotic.eu/) _(leader)_
    - [Faculty of Engineering, University of Porto (FEUP)](https://www.up.pt/feup/en/)
    - [INESC TEC - Institute for Systems and Computer Engineering, Technology and Science](https://www.inesctec.pt/en/)
    - [STAR](https://starinstitute.pt/)
    - [Kaizen](https://kaizen.com/pt-pt/)
    - [Institute for Systems and Robotics (ISR)-Coimbra](https://www.isr.uc.pt/)
- **Timeline:** October 2021 - December 2025
- **Duration:** 51 months
- **URL:**
  [https://transparencia.gov.pt/en/fundos-europeus/prr/beneficiarios-projetos/projeto/02/C05-i01.02/2022.PC644867037-00000013/](https://transparencia.gov.pt/en/fundos-europeus/prr/beneficiarios-projetos/projeto/02/C05-i01.02/2022.PC644867037-00000013/)
