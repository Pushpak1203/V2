# 🚗 Autonomous Multi-Agent V2V Simulation (MetaDrive)

A **multi-agent autonomous driving simulation** built on **MetaDrive**, enabling **Vehicle-to-Vehicle (V2V) communication**, **cooperative decision-making**, and **secure message exchange**.  
The project evaluates how shared situational awareness improves **safety**, **coordination**, and **responsiveness** among autonomous vehicles.

---

## ✨ Key Features
- Multi-agent autonomous driving simulation
- Secure V2V communication (encryption enabled)
- Hybrid A* based path planning
- Intelligent Driver Model (IDM) integration
- Config-driven simulation (YAML)
- Detailed logging and analysis support

---

## 📁 Project Structure

```text
.
├── communication/          # V2V communication & security
│   ├── broadcaster.py
│   ├── receiver.py
│   ├── encryption_utils.py
│   └── logging_config.py
│
├── decision_engine/        # Autonomous decision making
│   ├── planner.py
│   ├── hybrid_astar_planner.py
│   └── response_planner.py
│
├── metadrive_env/          # MetaDrive environment handling
│   ├── env_manager.py
│   └── env_manager_with_idm.py
│
├── config/                 # Configuration files
│   └── config.yaml
│
├── logs/                   # Runtime logs
│   ├── communication.log
│   └── sim_log.txt
│
├── Video Demo/             # Simulation demo videos
├── Research Paper/         # Reference research papers
│
├── main.py                 # Main entry point
├── main_autonomous_complex.py
├── main_autonomous_idm.py
├── drive_in_real_env.py
│
├── requirements.txt        # Dependencies
└── secret.key              # Encryption key (not for public repos)
```

---

## 🧠 System Architecture (High Level)

```
[MetaDrive Environment]
        ↓
[Autonomous Agents]
        ↓
[Decision Engine]
        ↓
[V2V Communication Layer]
        ↓
[Cooperative Actions]
```

---

## ⚙️ Installation

```bash
git clone https://github.com/your-username/your-repo-name.git
cd your-repo-name
pip install -r requirements.txt
```

---

## ▶️ Usage

Run the default multi-agent simulation:
```bash
python main.py
```

Run IDM-based simulation:
```bash
python main_autonomous_idm.py
```

Run complex cooperative scenario:
```bash
python main_autonomous_complex.py
```

---

## 🔐 Security Notice
`secret.key` is used for encrypted V2V communication.  
**Do NOT commit this file to public repositories.** Add it to `.gitignore`.

---

## 📊 Logs & Evaluation
- Communication events → `logs/communication.log`
- Simulation behavior → `logs/sim_log.txt`

These logs are used for **latency analysis**, **collision evaluation**, and **coordination assessment**.

---

## 🚀 Future Enhancements
- V2I / V2X communication support
- Reinforcement learning-based agents
- Latency and packet-loss modeling
- Real-world traffic dataset integration

---

## 📄 Documentation
- Project Report (PDF)
- Research Papers
- Demo Videos

---

## 📌 License
This project is intended for **academic and research purposes**.
