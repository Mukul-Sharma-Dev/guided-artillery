# SIH 2026 Presentation Deck Guide

## Slide 1: Title & Team
- **Visual:** Project title "Precision Guidance Kit (PGK) & Smart Multi-Mode Electronic Fuze", Team Name, YIL Branding.
- **Talking Point:** "Good morning judges. We are [Team Name], presenting our solution for upgrading unguided 155mm artillery shells into precision-guided munitions compatible with Yantra India Limited's manufacturing lines."

## Slide 2: Problem Analysis
- **Visual:** Artillery CEP comparison diagram (150m vs 30m).
- **Talking Point:** "Standard 155mm shells have a CEP of 150-200m at max range. This requires multiple rounds to destroy a target, straining logistics and risking collateral damage."

## Slide 3: Our Solution Overview
- **Visual:** 3D rendering or concept art of the PGK screwed into a standard shell.
- **Talking Point:** "Our PGK replaces the standard fuze. It features a de-spin bearing, 4-canard aerodynamic steering, and a smart multi-mode fuze, dropping CEP to under 30 meters."

## Slide 4: System Architecture
- **Visual:** System block diagram from SYSTEM_ARCHITECTURE.md.
- **Talking Point:** "The architecture centers on an extreme-G tolerant flight computer integrating GPS and MEMS IMU, powered by a setback-activated thermal battery."

## Slide 5: Canard Actuation Design
- **Visual:** De-spin bearing mechanics and 4-canard layout.
- **Talking Point:** "To steer a shell spinning at 250 revs/sec, our de-spin bearing allows the forward fin section to stabilize. The canards deflect up to ±15° at 300°/s."

## Slide 6: GNC Algorithm
- **Visual:** EKF equations block and Proportional Navigation diagram.
- **Talking Point:** "We fuse GPS and IMU data using a 6-state Extended Kalman Filter. The guidance uses Proportional Navigation to generate pitch/yaw commands."

## Slide 7: Multi-Mode Electronic Fuze
- **Visual:** Fuze State Machine (SAFE -> ARMING -> ARMED -> ACTIVE -> DET).
- **Talking Point:** "Safety is paramount. The fuze relies on dual-environment arming (15,000g setback + sustained spin) complying with MIL-STD-1316."

## Slide 8: Simulation Results
- **Visual:** Trajectory plot comparing guided vs unguided paths.
- **Talking Point:** "Our simulation models the 43.2kg shell at 820m/s muzzle velocity. The guided trajectory actively corrects cross-track error induced by wind and drift."

## Slide 9: Monte Carlo Validation
- **Visual:** Scatter plot of impact points showing 90% within 30m radius.
- **Talking Point:** "Running hundreds of Monte Carlo simulations with varied wind, GPS noise (σ=2m), and IMU bias confirms a CEP of strictly less than 30 meters."

## Slide 10: High-G Hardening & Manufacturing
- **Visual:** Potting compound diagram, compressive PCB layout.
- **Talking Point:** "Surviving 20,000g requires eliminating connectors, using rigid-flex PCBs, and full epoxy potting. The design is modular for drop-in compatibility at YIL facilities."

## Slide 11: Live Demo
- **Visual:** HIL Hardware setup running the python simulator.
- **Talking Point:** "Here is our Hardware-in-the-Loop demonstrator. As the simulation launches, you can see the fuze state LEDs change and the canard servos actuating to correct the trajectory."

## Slide 12: Conclusion & Future Roadmap
- **Visual:** Timeline to TRL 7 (3-5 years).
- **Talking Point:** "We aim to deliver a $5000 unit cost at scale. Future work includes SAASM anti-jamming GPS and live-fire structural testing. Thank you."

---

## Judge Q&A Battlecards

**Q: How do canards steer a spinning shell?**
A: A de-spin bearing mechanically decouples the forward guidance section from the main shell body. An alternator controls the roll rate of the forward section, allowing the canards to exert a stable directional force.

**Q: How does electronics survive 15,000g?**
A: We use a compressive PCB layout where forces push components into the board. We eliminate heavy components and connectors, and the entire assembly is potted in a hard, glass-filled epoxy resin.

**Q: What if GPS is jammed?**
A: The system falls back to IMU-only dead reckoning. While CEP degrades from <30m to approximately 45-50m depending on time of flight, it still performs significantly better than unguided munitions.

**Q: What's the power budget?**
A: The system uses a setback-activated thermal battery providing ~16W peak power for a maximum flight time of 120 seconds.

**Q: How is this different from M1156 PGK?**
A: Our design integrates advanced multi-mode fuzing directly with the guidance kit and is specifically tailored for YIL's manufacturing capabilities and the Indian Army's environmental requirements.

**Q: Cost per unit?**
A: At scale, we target <$5000 USD, which is highly cost-effective compared to specialized guided shells like Excalibur (>$100k).

**Q: What's the Technology Readiness Level?**
A: This demonstrator represents TRL 3-4 (analytical and experimental critical function proof of concept).

**Q: Can this work with Indian Army's existing 155mm guns?**
A: Yes, it uses the standard NATO 2-inch fuze well thread, making it a drop-in replacement for standard fuzes on existing shells like the Bofors or Dhanush.

**Q: What about anti-jamming?**
A: Future iterations will include a SAASM (Selective Availability Anti-Spoofing Module) compatible GPS receiver and anti-jamming antenna arrays.

**Q: Spin rate effects on electronics?**
A: The de-spin bearing significantly reduces the spin rate experienced by the forward electronics, mitigating extreme centrifugal forces.

**Q: Safety certification path?**
A: The fuze state machine uses independent safety interlocks (setback and spin) meeting MIL-STD-1316 and STANAG 4187 requirements.

**Q: Manufacturing at YIL?**
A: The modular form factor allows it to be screwed onto existing shell bodies on the current YIL assembly lines without modifying the explosive filling process.

**Q: What sensors specifically?**
A: We specify high-G tolerant MEMS like the Analog Devices ADIS16490 IMU and ADXL001 accelerometers, proven in high-shock environments.

**Q: Timeline to production?**
A: Moving from TRL 4 to TRL 7/8 (live fire qualification) typically takes 3-5 years with adequate funding and range access.

**Q: Comparison with Excalibur/Krasnopol?**
A: Excalibur is a fully custom guided shell with a base bleed and glide capability, offering longer range and smaller CEP but at 20x the cost. Our PGK is a cheap, bolt-on upgrade for the massive stockpile of "dumb" rounds.
