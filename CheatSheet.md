## Jetson Command Line Cheat Sheet

A comprehensive guide with useful commands for NVIDIA Jetson.

---

### 1. General Information

| Command            | Purpose                     |
|--------------------|-----------------------------|
| `uname -m`         | Check system architecture   |
| `lsb_release -a`   | Check JetPack/L4T version   |
| `nvcc --version`   | Check CUDA version          |

---

### 2. Network Management

#### Create Wi-Fi Hotspot

```bash
nmcli dev wifi hotspot ifname wlan0 ssid mxck0022 password mxck0022
```

#### List Available Networks

```bash
nmcli dev wifi list
```

#### Connect to Wi-Fi Network

```bash
nmcli dev wifi connect NETWORK_SSID password NETWORK_PASSWORD
```

---

### 3. SSH Management

#### Check if SSH is Running

```bash
systemctl status ssh
```

#### Start SSH Service

```bash
sudo systemctl start ssh
```

#### Enable SSH Service (start on boot)

```bash
sudo systemctl enable ssh
```

---

### 4. Process Management

#### List Running Processes

```bash
ps aux
```

#### Kill Process by ID

```bash
sudo kill -9 PROCESS_ID
```

#### Kill Process by Name

```bash
sudo pkill PROCESS_NAME
```

---

### 5. Port Management

#### Check Used Ports

```bash
sudo netstat -plnt
```

---

### 6. System Date and Time

#### Set Date and Time Manually

```bash
sudo date -s "YYYY-MM-DD HH:MM:SS"
```

---

### 7. Power and Resource Management

#### Check Current Power Mode

```bash
sudo nvpmodel -q
```

#### Check GPU, CPU, and Memory Usage

```bash
tegrastats
```

---

### 8. NoMachine Installation and Usage

#### Download and Install NoMachine

```bash
wget https://download.nomachine.com/download/8.16/Arm/nomachine_8.16.1_1_arm64.deb
sudo dpkg -i nomachine_8.16.1_1_arm64.deb
sudo apt-get install -f
```

#### Check NoMachine Status

```bash
sudo /etc/NX/nxserver --status
```

#### Check NoMachine Port Usage

```bash
sudo netstat -plnt | grep nx
```

#### Start/Stop/Restart NoMachine

```bash
sudo /etc/NX/nxserver --start
sudo /etc/NX/nxserver --stop
sudo /etc/NX/nxserver --restart
```

---

You're now equipped with essential commands for efficient Jetson device management! 🚀

