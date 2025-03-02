# Integrating Multimodal Perception into Ground Mobile Robots

!!! Abstract

    Multimodal perception systems enhance the robustness and adaptability
    of autonomous mobile robots by integrating heterogeneous sensor modalities,
    improving long-term localisation and mapping in dynamic environments and
    human-robot interaction. Current mobile platforms often focus on specific
    sensor configurations and prioritise cost-effectiveness, possibly limiting
    the flexibility of the user to extend the original robots further.
    This paper presents a methodology to integrate multimodal perception into a
    ground mobile platform, incorporating wheel odometry, 2D laser scanners,
    3D Light Detection and Ranging (LiDAR), and RGBD cameras.
    The methodology describes the electronics design to power devices, firmware,
    computation and networking architecture aspects, and mechanical mounting
    for the sensory system based on 3D printing, laser cutting, and
    bending metal sheet processes. Experiments demonstrate the usage of the
    revised platform in 2D and 3D localisation and mapping and pallet pocket
    estimation applications. All the documentation and designs are accessible
    in a public repository.

    **Keywords:**
    Light Detection and Ranging (LiDAR),
    mobile robot,
    multimodal perception,
    open-source,
    RGBD camera.

This repository contains all the documentation associated with the modifications
made by [INESC TEC](https://www.inesctec.pt/en/) on the
[Hangfa Discovery Q2](https://www.hangfa-europe.com/en/omni-robot/discovery)
mobile platform in order to be compatible with multimodal perception. These
modifications also enable the platform to be integrated in the
[Robot Operating System (ROS)](https://ros.org/), facilitating its usage for
research topics such as perception, localisation and mapping, multi-robot
coordination (when more than one platform is available to the user), Artificial
Intelligence (AI) applied on autonomous mobile robotics, among other topics.

Furthermore, the main goals of this website and the respective GitHub repository
is to help researchers interested in modifying their Hangfa mobile platforms or
applying all the modifications that we have made to other similarly small mobile
platforms. As a result, the website includes the following information:

- **[Platform](content/discovery-q2.md):** brief presentation of the
  [Hangfa Discovery Q2](https://www.hangfa-europe.com/en/omni-robot/discovery)
  robot mobile platform
- **[Bill Of Materials (BOM)](content/bom.md):** summary on the components used for
  the modifications to the platform
- **[Electronics](content/electronics/index.md):** presentation of the electronics redesign
  (battery management and power budgets, motor drivers, encoders reading, and
  external DC power output for the user)
- **[Single Board Computer (SBC)](content/sbc/index.md):** computing units
  considered in the work and their configuration in terms of Operating System
  (OS), Robot Operating System (ROS) setup, configuring remote access, set up
  development environment, and configuring the firmware communication
- **[Network](content/):** TBC
- **[Sensors](content/):** TBC
- ... _To Be Completed (TBC)_

Lastly, this work is within the scope of the
[Mobile Robotics Development Team (MRDT)](https://gitlab.inesctec.pt/mrdt/) in
the national project
[GreenAuto: Green innovation for the Automotive Industry](https://transparencia.gov.pt/en/fundos-europeus/prr/beneficiarios-projetos/projeto/02/C05-i01.02/2022.PC644867037-00000013/). [MRDT](https://gitlab.inesctec.pt/mrdt/) team is a
Research & Development (R&D) team from the
[CRIIS - Centre for Robotics in Industry and Intelligent Systems](https://www.inesctec.pt/en/centres/criis)
at the [iiLab - Industry and Innovation Laboratory](https://www.inesctec.pt/en/laboratories/iilab-industry-and-innovation-lab).

## Videos

<iframe width="560" height="315" src="https://www.youtube.com/embed/videoseries?si=M_ykKm2IiWBJxEFs&amp;list=PLvp8fJUEPxYSkKsOrCN5FzjuhhSfVgSuR" title="YouTube video player" frameborder="0" allow="accelerometer; autoplay; clipboard-write; encrypted-media; gyroscope; picture-in-picture; web-share" referrerpolicy="strict-origin-when-cross-origin" allowfullscreen></iframe>

- **How To Watch**
    1. Press the :material-play: Play button
    2. Use the :material-skip-previous: Previous and :material-skip-next: Next
       buttons to navigate through the playlist
- **Playlist YouTube Link:**
  [https://youtube.com/playlist?list=PLvp8fJUEPxYSkKsOrCN5FzjuhhSfVgSuR&si=SDjPObDUzQucRvuN](https://youtube.com/playlist?list=PLvp8fJUEPxYSkKsOrCN5FzjuhhSfVgSuR&si=SDjPObDUzQucRvuN)

## Contacts

If you have any questions or you want to know more about this work, please
contact one of the following contributors:

- Ricardo B. Sousa
  ([rbs@fe.up.pt](mailto:rbs@fe.up.pt))
  _(Corresponding Author)_
  [:fontawesome-brands-github:](https://github.com/sousarbarb/),
  [:fontawesome-brands-gitlab:](https://gitlab.com/sousarbarb/),
  [:fontawesome-brands-gitlab:](https://gitlab.inesctec.pt/ricardo.b.sousa),
  [:fontawesome-brands-orcid:](https://orcid.org/0000-0003-4537-5095),
  [:fontawesome-brands-google-scholar:](https://scholar.google.pt/citations?user=Bz2FMqYAAAAJ),
  [:fontawesome-brands-linkedin:](https://www.linkedin.com/in/sousa-ricardob/),
  [:fontawesome-brands-youtube:](https://www.youtube.com/channel/UCXTR8mMlG0VOC_06PKg5KBQ)
- Héber Miguel Sobreira
  ([heber.m.sobreira@inesctec.pt](mailto:heber.m.sobreira@inesctec.pt))
  [:fontawesome-brands-github:](https://github.com/HeberSobreira),
  [:fontawesome-brands-gitlab:](https://gitlab.inesctec.pt/heber.m.sobreira/),
  [:fontawesome-brands-orcid:](https://orcid.org/0000-0002-8055-1093),
  [:fontawesome-brands-google-scholar:](https://scholar.google.pt/citations?user=iNhGcpsAAAAJ)
- João G. Martins
  ([joao.g.martins@inesctec.pt](mailto:joao.g.martins@inesctec.pt))
  [:fontawesome-brands-github:](https://github.com/Joao-G-Martins),
  [:fontawesome-brands-orcid:](https://orcid.org/0000-0002-6567-4802)
- Paulo G. Costa
  ([paco@fe.up.pt](mailto:paco@fe.up.pt))
  [:fontawesome-brands-github:](https://github.com/P33a),
  [:fontawesome-brands-orcid:](https://orcid.org/0000-0002-4846-271X),
  [:fontawesome-brands-google-scholar:](https://scholar.google.pt/citations?user=7Iz8fKcAAAAJ)
- Manuel F. Silva
  ([mss@isep.ipp.pt](mailto:mss@isep.ipp.pt))
  [:fontawesome-brands-orcid:](https://orcid.org/0000-0002-0593-2865),
  [:fontawesome-brands-google-scholar:](https://scholar.google.pt/citations?user=2EFVZ-AAAAAJ)
- António Paulo Moreira
  ([amoreira@fe.up.pt](mailto:amoreira@fe.up.pt))
  [:fontawesome-brands-orcid:](https://orcid.org/0000-0001-8573-3147),
  [:fontawesome-brands-google-scholar:](https://scholar.google.pt/citations?user=eL0gHLoAAAAJ)

## Institutions

<div class="grid cards" markdown>

- [![INESC TEC Logo](assets/logo/inesctec_logo_color-rgb.png)](https://www.inesctec.pt/en/)
- [![FEUP Logo](assets/logo/feup_logo_oficial.png)](https://sigarra.up.pt/feup/en/)

</div>

## Acknowledgements

<div class="grid cards" markdown>

- **[5dpo Robotics Team](https://5dpo.github.io/)**
- [![Amorins & Silva Logo](assets/logo/amorins-e-silva_logo.png)](https://amorinsesilva.pt/)
- [![Hangfa Robotics Europe Logo](assets/logo/hangfa-europe_logo.png)](https://www.hangfa-europe.com/)
- [![LattePanda Logo](assets/logo/lattepanda_logo.png)](https://www.lattepanda.com/)

</div>

## Funding

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

## Citation

**Plain Text**

R.B. Sousa, H.M. Sobreira, J.G. Martins, P.G. Costa, M.F. Silva and A.P.
Moreira,
"Integrating Multimodal Perception into Ground Mobile Robots,"
_2025 IEEE International Conference on Autonomous Robot Systems and_
_Competitions (ICARSC2025)_, Madeira, Portugal, 2025, pp. TBD, doi: TBD
[Manuscript accepted for publication].
[[github]](https://github.com/sousarbarb/inesctec_mrdt_hangfa_discovery_q2)
[[preprint]](http://doi.org/10.13140/RG.2.2.29381.15845/1)

**BibTex**

```bibtex
@INPROCEEDINGS{sousa2025icarsc,
  author    = {Ricardo B. Sousa and Héber Miguel Sobreira and João G. Martins and Paulo G. Costa and Manuel F. Silva and António P. Moreira},
  booktitle = {2025 IEEE International Conference on Autonomous Robot Systems and Competitions (ICARSC)},
  title     = {Integrating Multimodal Perception into Ground Mobile Robots},
  year      = {2025},
  volume    = {},
  number    = {},
  pages     = {--},
  doi       = {},
  note      = {Manuscript accepted for publication},}
```
