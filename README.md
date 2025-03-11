# Brake Rotor Thermal Simulation (Lumped-Body Model)

1. Model **temperature rise** due to repeated braking event
2. Include **convection** and **radiation** to ambient
3. Incorporate simple **conduction** to the wheel hub
4. Use **multiple rotor materials** with **temperature-dependent** properties

---

## Table of Contents

1. [Setup & Requirements](#setup--requirements)
2. [How to Run](#how-to-run)
3. [Input Parameters](#input-parameters)
4. [Extending or Adjusting the Model](#extending-or-adjusting-the-model)
5. [Key Assumptions](#key-assumptions)
6. [References](#references)

---

## Setup & Requirements

**Dependencies**:

- `numpy` (arrays and math)
- `matplotlib` (plotting)

Install with:

```bash
pip install numpy matplotlib
```

---

## How to Run

**Execute**:

```bash
python BRAKE_THERMAL_SIM.py
```

The script will:

- Generate random events (omega_init, omega_final, t_brake, t_cool)
- Simulate rotor temperatures for each rotor
- Print out the peak temperature reached by each rotor
- Show plot of front and rear rotor temperature along with a braking vs cooling flag

---

## Input Parameters

## 1. `rotor_pairs`

A list of lists, where each inner list represents **two rotors** (front and rear, or left and right). Each rotor is a dictionary with:

- **`mass_disc` (kg)**: Lumped disc mass.
- **`area_disc` (m²)**: Effective surface area for convection/radiation.
- **`fraction_into_disc` (-)**: Fraction of braking heat that goes into the rotor (0–1).
- **`I_wheel` (kg·m²)**: Rotational inertia of the wheel-rotor assembly (for computing ΔE).
- **`emissivity` (-)**: Surface emissivity for radiative heat loss.
- **`material_type` (int)**: Identifier used by `get_material_properties` to retrieve density, conductivity, etc.

**Example**:

```python
rotor_pairs = [
    [
        {
            "mass_disc": 0.5289,
            "area_disc": 0.055,
            "fraction_into_disc": 0.9,
            "I_wheel": 5.0213,
            "emissivity": 0.6,
            "material_type": 2
        },
        {
            "mass_disc": 0.3778,
            "area_disc": 0.0347,
            "fraction_into_disc": 0.9,
            "I_wheel": 3.542,
            "emissivity": 0.6,
            "material_type": 2
        }
    ],
    [
        {
            "mass_disc": 0.500,
            "area_disc": 0.050,
            "fraction_into_disc": 0.9,
            "I_wheel": 4.8,
            "emissivity": 0.75,
            "material_type": 1
        },
        {
            "mass_disc": 0.360,
            "area_disc": 0.033,
            "fraction_into_disc": 0.9,
            "I_wheel": 3.4,
            "emissivity": 0.75,
            "material_type": 1
        }
    ]
]
```

**IMPORTANT**

- In `get_material_properties`, you store more materials or expand the piecewise
- also assign **`material_names`** after storing materials
- and assign **`material_type`** in `rotor_pairs` appropriately

---

## Extending or Adjusting the Model

### 1. Adding Aerodynamic Drag & Rolling Resistance

- all deceleration is assumed to come from the brakes.
- **Option**: subtract a “drag fraction” from each braking event’s total energy to account for aerodynamic drag and rolling resistance.

### 2. Refining Conduction to Hub

- consider an approach that includes the **hub temperature** changing over time.

### 3. Pad and Rotor Heat Partition

- instead of a constant fraction of the total brake heat going into the rotor, use thermal properties of the **pad vs. rotor** and contact resistances.
- **Current**: **constant fraction** near 0.9. Adjust or replace depending on data on pad conductivity, cooling rates, or caliper heat flow.

### 4. Add Real Sensor Data

- **Current**: Simulates random breaking events (50 - 60 mph) and adds linear fit between each event.
- implement with real wheel speed sensor data

---

## Key Assumptions

### 1. Lumped Mass / Lumped Body Approximation

- We assume **no temperature gradients** within the rotor; the **entire disc** is considered at a **single, uniform temperature** at each time step.
- This simplifies the math but **underestimates local “hot spots”**. It’s best for **comparing trends** rather than for exact peak surface temperature.

### 2. Aerodynamic Drag & Rolling Resistance

- The provided script does **not** currently subtract a separate portion for vehicle deceleration due to drag or rolling resistance.

### 3. Conduction to the Wheel Hub

- A simple conduction model is used:
  $$
  \dot{Q}_{\text{cond}} = k_{\text{cond}} \cdot \text{area} \cdot (T_{\text{rotor}} - T_{\text{hub}})
  $$
- **`k_cond` and `T_hub`** are **user-defined** placeholders. This does **not** account for actual geometry or conduction paths.
- potentially adjust these parameters (with a separate hub temperature model).

### 4. Fraction of Brake Energy into the Disc

- Each rotor has a `fraction_into_disc` property (often set near 0.85–0.95 in many references).
- By default, set to **0.9**: ~90% of the frictional heat is assumed to enter the rotor instead of the brake pads or caliper.
- tune this fraction based on your IR sensor data?

### 5. Ambient Temperature

- The environment is assumed to be at a **fixed** temperature (25 °C).

---

## References

**Medina, Luis.**  
 _Estimating the Working Temperature of a Brake Disc._  
 Published on **Made After Hours**, 16 July 2023.  
 [Discusses the lumped-body approach to brake rotor thermal modeling, heat partitioning, and iterative calculation methods.](https://madeafterhours.com)

> This simulation code draws on ideas similar to those presented by Luis Medina in his blog post. In particular, the **lumped-body approach**, partitioning heat between pad and rotor, and using iterative time steps for **braking** vs. **cooling** phases are directly inspired by his work.
