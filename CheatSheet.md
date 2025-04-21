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

#### Stop Hotspot

To stop a running hotspot (so the Jetson can connect to a normal Wi-Fi network):

1. List all active and saved connections:
   ```bash
   nmcli connection show
   ```
2. Find the hotspot connection name (e.g. `Hotspot`, or similar)
3. Bring the hotspot connection down:
   ```bash
   nmcli connection down <name_of_hotspot>
   ```
   Example:
   ```bash
   nmcli connection down Hotspot
   ```

---


### 3. SSH Management

#### Manage SSH Server (Incoming Connections)

These commands manage the SSH server daemon (`sshd`) on the machine, controlling whether others can connect *to* it. Run these on the machine you want to connect *to* (e.g., the Jetson).

##### Check if SSH Server is Running

```bash
systemctl status ssh
```

##### Start SSH Server Service

```bash
sudo systemctl start ssh
```

##### Enable SSH Server Service (start on boot)

```bash
sudo systemctl enable ssh
```

---

#### Manage SSH Client Agent (Outgoing Connections)

These commands manage the SSH agent (`ssh-agent`) on your *local* machine (the one you are connecting *from*). It helps manage your private keys for authenticating *to* other servers.

##### Start SSH Agent for Current Session

```bash
eval "$(ssh-agent -s)"
```

* **Brief Explanation:** This command starts the `ssh-agent` program in the background for your current shell session. The agent can hold your decrypted private SSH keys in memory.

##### Add a Private Key to the Agent

After starting the agent, you can add specific private keys to it using the `ssh-add` command.

```bash
ssh-add ~/.ssh/github_key
```

* **Example Explanation:** This command adds the private key located at `~/.ssh/github_key` to the running agent. If this key is protected by a passphrase, `ssh-add` will ask you for it *once*. After the key is added, the agent can provide it automatically when you SSH to hosts that accept the corresponding public key (like GitHub, or your Jetsons if configured), eliminating the need to re-enter the passphrase for that session. You can run `ssh-add` for multiple keys.

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

##### Check Status of Network Time Sync

```bash
timedatectl status
```

##### Enable Auto Sync with Internet Time (Turn ON)

```bash
sudo timedatectl set-ntp true
```

##### Disable Auto Sync with Internet Time (Turn OFF)

```bash
sudo timedatectl set-ntp false
```

#### Change Keyboard Layout (No Reboot Needed)  
```bash
sudo loadkeys us  # Example: Set to US layout
```

---

### 7. Power and Resource Management

#### Check Current Power Mode

```bash
sudo nvpmodel -q
```

#### List Available Power Modes (Configs)  
To list all available power modes defined in your system configuration file:  
```bash
cat /etc/nvpmodel.conf
```
#### Available Power Modes on Xavier NX  
The most common modes for Xavier NX are:

| Mode ID | Name            | Description                   |
|---------|-----------------|------------------------------|
| 0       | MODE_15W_2CORE   | 2 cores, moderate performance, 15W limit. |
| 1       | MODE_15W_4CORE   | 4 cores, moderate performance, 15W limit. |
| 2       | MODE_15W_6CORE   | 6 cores, moderate performance, 15W limit. |
| 3       | MODE_10W_2CORE   | 2 cores, low power, 10W limit. |
| 4       | MODE_10W_4CORE   | 4 cores, low power, 10W limit. |
| 5       | MODE_10W_DESKTOP | 4 cores, low power, optimized for desktop. |
| 6       | MODE_20W_2CORE   | 2 cores, high performance, 20W limit. |
| 7       | MODE_20W_4CORE   | 4 cores, high performance, 20W limit. |
| 8       | MODE_20W_6CORE   | 6 cores, high performance, 20W limit. |

#### Change Power Mode  
To change to a different power mode, use the following command with the appropriate ID:  
```bash
sudo nvpmodel -m <mode_id>
```
Example (Switch to **MODE_20W_6CORE**):  
```bash
sudo nvpmodel -m 8
```

#### Check GPU, CPU, and Memory Usage

```bash
tegrastats
```

#### Using jtop (If Installed)
```bash
jtop
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

### 9. Listing Permissions with `ls` and `stat`  

#### List Permissions (Standard)  
```bash
ls -l
```

#### List Permissions (Octal Format)  
```bash
stat -c "%a %n" *
```

