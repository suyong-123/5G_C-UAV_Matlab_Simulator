# 5G C-UAV MATLAB Simulator

This repository provides a **modular and extensible 5G network simulation framework** for connected UAVs, implemented in MATLAB.  
It supports mobility management, scalable UE deployment, and integration with multiple transport-layer protocols (TCP, UDP, QUIC, MP-QUIC).  
Reinforcement learning (RL) is incorporated for intelligent handover decision-making.

---

## Publication

📄 *A Modular and Scalable Simulator for Connected UAVs Communication in 5G Networks*  
Accepted by **MSWiM 2025**. Full paper available on [arXiv](https://arxiv.org/abs/2509.00868).

---

## Project Structure

- **Mobility Management**: `HandoverManager.m`, `generate_fixed_path.m`, `base_network.m`  
- **Scalability Testing**: `scalability_test.m`  
- **Transport Protocols**: `tcp_matlab.m`, `scalability_test.m` , `quic_matlab.m`, `mpquic_matlab.m`  
- **Handover Strategies**: `main_training.m`, `main_evaluate.m`  

---

## Key Features

- Modular and scalable MATLAB-based simulator  
- UAV trajectory generation (fixed and random)  
- Transport protocol flexibility: TCP, UDP, QUIC, MP-QUIC  
- RL-based handover compared with rule-based strategies  
- Multi-UE scalability experiments  

---

## Contributors & Roles

| Contributor                     | Role                                            |
| ------------------------------- | ----------------------------------------------- |
| **Yong Su**                     | Project lead; overall design                    |
| **Yiyi Chen**, **Shenghong Yi** | Implementation of handover strategy comparisons |
| **Hui Feng**                    | Guidance on experiment design and methodology   |

---

## Requirements

- MATLAB R2024b or newer  
- Python (for external RL integration)  
- [quic-go](https://github.com/lucas-clemente/quic-go) for QUIC/MP-QUIC  

---

## Contact

For academic collaborations or questions:  
**Yong Su** – [suy21@m.fudan.edu.cn]  