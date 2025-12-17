# BLE Adaptive Frequency Hopping (AFH) Simulator

## Overview

This project implements a **MATLAB-based simulator for Bluetooth Low Energy (BLE) Adaptive Frequency Hopping (AFH)** under realistic interference conditions in the 2.4 GHz ISM band.

The simulator models:

* BLE packet transmissions with retransmissions and latency
* Multiple AFH **channel selection algorithms**
* Multiple AFH **channel classification algorithms**
* Time-varying, heterogeneous interferers (Wi-Fi–like, narrowband, bursty)
* Packet loss, throughput, latency, and retransmission statistics
* Detailed visualization of channel state evolution over time

The primary entry point is:

```
ble_afh_sim.m
```

Running this function executes a complete simulation, prints a summary, and generates diagnostic plots.

---

## Requirements

* MATLAB R2021a or newer (older versions may work but are untested)
* No external toolboxes required beyond base MATLAB

---

## Quick Start

1. Place `ble_afh_sim.m` on your MATLAB path.
2. From the MATLAB command window, run:

   ```matlab
   ble_afh_sim
   ```
3. The simulator will:

   * Run a full BLE AFH simulation
   * Print a summary of performance metrics
   * Open multiple figures showing channel behavior, retransmissions, and latency

---

## High-Level Simulation Flow

Each simulation proceeds as follows:

1. **Initialize parameters**

   * BLE PHY and connection parameters
   * Noise and interference configuration
   * AFH algorithm selection

2. **Generate packets**

   * Each packet may require multiple retransmissions
   * Connection interval timing is respected

3. **Select a BLE data channel**

   * According to the configured AFH *selection* algorithm

4. **Apply interference and noise**

   * Interferers may turn on/off over time
   * Energy is computed per channel

5. **Determine packet success/failure**

   * BER-based model using SNR and packet length

6. **Update channel classification**

   * Channels may be marked bad or recovered

7. **Collect metrics**

   * Throughput, packet loss, latency, retransmissions

---

## Configurable Parameters

### 1. AFH Algorithm Selection

Two global variables control the AFH behavior.
These can be set **before** running `ble_afh_sim`.

#### AFH Selection Algorithm (`selAlg`)

```matlab
selAlg = 2;
```

| Value | Description                                          |
| ----: | ---------------------------------------------------- |
|     1 | Simple hop over allowed channels                     |
|     2 | BLE-compliant pseudo-random channel selection        |
|     3 | Custom selector (currently identical to Algorithm 1) |

#### AFH Classification Algorithm (`classAlg`)

```matlab
classAlg = 4;
```

| Value | Description                                     |
| ----: | ----------------------------------------------- |
|     0 | No classification (baseline)                    |
|     1 | Fixed-window packet failure rate                |
|     2 | EWMA of packet failure fraction                 |
|     3 | Energy / RSSI-based classification              |
|     4 | **Combined EWMA + energy (recommended)**        |
|     5 | unused, Predictive (packet-failure-based, experimental) |
|     6 | unused, Predictive (energy-based, experimental)         |

> **Recommended default:**
> `selAlg = 2`, `classAlg = 4`

---

### 2. Simulation Parameters

Key parameters defined near the top of the file:

```matlab
sim.totalPackets      = 20000;
sim.payloadBytes     = 20;
sim.maxRetransmit    = 24;
sim.connInterval_ms  = 7.5;
```

* Increasing `totalPackets` improves statistical stability
* `connInterval_ms` controls latency and throughput
* `maxRetransmit` affects disconnection probability

---

### 3. Noise and Interference Configuration

```matlab
chan.noise_dBm              = -90;
chan.signal_power_dBm       = -40;
chan.interference_power_dBm = -40;
```

Interferers are defined as a struct array with:

* Center channel
* Bandwidth (in BLE channels)
* Mean power
* On/off probability
* Burst duration

This allows modeling:

* Wideband Wi-Fi interference
* Narrowband periodic interferers
* Bursty and continuous noise sources

---

## Output Metrics

At the end of the simulation, the following metrics are printed:

* **Packet loss rate**
* **Throughput (bps and KBps)**
* **Retransmission ratio**
* **Median, p95, p99, and max latency**
* **Number of disconnections**

Example output:

```
--- Summary ---
AFH selection: 2 pseudo-random
AFH classification: 4 combined (EWMA+energy)
Packets: 20000, Success: 19321, Retransmissions: 8213 (ratio=0.411)
Throughput: 24123.4 bps, 3.015 KBps
Median latency: 7.50 ms, p95: 15.00 ms, p99: 30.00 ms, max: 187.50 ms
```

---

## Generated Figures

The simulator automatically generates four figures:

### 1. Channel State Timeline

* Heatmap of **allowed vs excluded channels** over time
* Packet successes (green dots) and failures (red Xs)
* Useful for visualizing AFH behavior and recovery

### 2. Retransmissions per Packet

* Histogram (log scale)
* Shows reliability and contention effects

### 3. Latency CDF

* Empirical cumulative distribution of packet latency

### 4. Latency vs Percentile (Log Scale)

* Highlights tail latency behavior (p95, p99, worst-case)

---

## Predictive Algorithms (Experimental)

Classification algorithms 5 and 6 implement **predictive AFH**:

* They're unfinished, but would use past data to predict interference based on periodic trends

---

## Extending the Simulator

Possible extension points include:

* Adding new interferer models
* Implementing custom AFH classifiers
* Studying power consumption effects
* Integrating real-world measurement traces
* Evaluating machine learning–based classifiers



