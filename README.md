# 🚚 Vehicle Routing Problem (VRP) — KNN & Optimization Methods

This project focuses on solving the **Vehicle Routing Problem (VRP)**.  
The solution was initially built using a **K-Nearest Neighbour (KNN) heuristic** to create a baseline set of routes.  
Then, several optimization algorithms were applied to improve route cost, distance, and feasibility.

---

## 🔧 Main Concept
- Generate an initial VRP solution using **KNN heuristic**
- Improve the baseline solution with multiple **local optimization methods**
- Compare performance across different algorithms

---

## 🧠 Methods Implemented / Evaluated

### ✔ Construction Heuristic
- **K-Nearest Neighbour (KNN)** for initial route building

### ✔ Local Search Methods
- **2-opt**
- **3-opt**
- **Swap**
- **Relocate**

### ✔ Metaheuristics
- **Tabu Search**
- **Simulated Annealing**
- **Variable Neighborhood Search (VNS)**

---

## 🎯 Goals
- Reduce total route distance  
- Minimize number of vehicles (when applicable)  
- Improve time or capacity feasibility  
- Compare algorithm performance  

---

## 🛠 Technologies
Python · NumPy · Matplotlib *(or whatever you used)*
