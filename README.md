# Robotic Arm Control Simulation

A Simulink-based project modeling and analyzing a 2-link robotic arm. Focused on comparing open-loop and closed-loop control using PID controllers. The simulation demonstrates core principles in robotic control, dynamics, and system stability.

---

## 🎯 Objectives

- Model dynamic behavior of a 2-joint robotic arm.
- Apply Laplace transforms to derive system transfer functions.
- Compare open-loop vs. closed-loop performance.
- Implement basic PID controllers to improve movement accuracy.

---

## ⚙️ System Overview

### 🧮 Mathematical Model:
- Torque Equation: τ = J * θ̈ + b * θ̇
- Transfer Function: G(s) = 1 / (J * s² + b * s)
- PID: u(t) = Kp * e(t) + Ki ∫e(t)dt + Kd * de(t)/dt

### PID Setup:
- Joint 1 and Joint 2 each use independent PID controllers.
- Transfer functions tested with and without feedback.

---

## 📉 Observations

### ✅ Open-Loop:
- No feedback → unstable motion.
- Joint 2 moves faster due to lower inertia.
- Final positions inaccurate.

### ✅ Closed-Loop:
- PID control improves stability and accuracy.
- End-effector still does not reach target smoothly.
- PID values not well tuned yet.

---

## ⚠️ Known Issues

- End-effector motion is unstable and imprecise.
- System oscillates and overshoots.
- PID tuning incomplete.
- Damping and inertia values need optimization.

---

## 🔧 Future Improvements

- Tune Kp, Ki, Kd for better performance.
- Increase damping values.
- Implement adaptive PID control with real-time feedback.
- Add graphical visualizations or animations of motion.

---

## 📁 Files

| File | Description |
|------|-------------|
| `robotic_arm_model.slx` | Simulink model (if available) |
| `roboticarmpresentation.pdf` | Full project presentation and results |
| `README.md` | Project overview |

---

## 🧪 Status

⚠️ **This project is a work in progress.**  
The core model is built, but control tuning and smooth motion are still under development.

---

## ✍️ Author

Made by [Özgün Deniz Sevilmiş](mailto:ozgundenizsevilmis@gmail.com) during Erasmus coursework on control systems and robotics.
