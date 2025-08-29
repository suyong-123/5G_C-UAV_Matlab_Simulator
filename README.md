# C-UAV Systems Simulation Platform

This repository contains a modular and extensible 5G network simulation framework for connected UAVs, implemented in MATLAB. The platform supports mobility management, scalable UE deployment, and integration with various transport-layer protocols including QUIC, MP-QUIC, TCP, and UDP. It also incorporates reinforcement learning (RL) for intelligent handover decision-making.

## Project Structure

### Toolbox Enhancements

- **internal_changes.diff**

- **wirelessnetwork_changes.diff**

  `internal_changes.diff`: Added disconnect and reconnect functions required for handover in each layer of the 5G protocol stack.

  `wirelessnetwork_changes.diff`: Major modifications focused on `nrUE` and `nrGNB`.

------

### Mobility Management

- **`handoverManager.m`**
   Mobility management module supporting multiple handover strategies (A3 event-based, DQN-based, etc.).
- **`generateFixedPath.m`**
   Script for generating fixed UAV trajectories to test handover performance under reproducible conditions.
- **`baseNetwork.m`**
   Defines the baseline 5G simulation environment, including gNBs and UEs.

------

### Scalability Testing

- **`scalable.m`**
   Supports multi-UE scenarios for testing the performance and scalability of the system.

------

### Transport Layer Protocol Integration

- **`multi-quic-client.m`**
   Integrates the QUIC protocol using a MATLAB interface to an external Go-based QUIC implementation.
- **`tcp_matlab.m`**
   Implements traditional TCP communication.
- **`mpquic_matlab.m`**
   Integrates Multipath-QUIC (MP-QUIC) to simulate simultaneous multi-link data flows.
- **`scalable.m`**
   Also supports simple UDP-based transmission for lightweight testing.

------

### Handover Strategy Comparison

- **`main_training_script.m`**
   Trains an RL-based handover agent (e.g., DQN/DDQN) using environment feedback and SINR measurements.
- **`main_evaluate_script.m`**
   Evaluates the performance of different handover strategies under identical conditions.

------

## Key Features

- Modular and scalable simulation design
- Support for fixed-path and random UAV trajectories
- Transport protocol flexibility: TCP, UDP, QUIC, MP-QUIC
- RL-based intelligent handover with comparison against conventional rules
- Multi-UE scalability testing

------

## Requirements

- MATLAB R2024b or newer 

- Python (for external RL agent integration)

- [quic-go](https://github.com/lucas-clemente/quic-go) for QUIC/MP-QUIC support

  

------

## Contact

For academic collaborations or questions, please contact:

**[Yong Su]**
 [suy21@m.fudan.edu.cn]