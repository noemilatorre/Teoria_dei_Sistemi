# ROBILAUT - Controllo del Moto di un Robot Mobile

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange?logo=mathworks)](https://www.mathworks.com/)
[![Control Systems](https://img.shields.io/badge/Control-MPC%20%26%20LQR-blue)](https://www.mathworks.com/products/control.html)
[![Status](https://img.shields.io/badge/Status-Completed-success)]()

Progetto di **Teoria dei Sistemi** per la progettazione del controllo del moto di un robot mobile di tipo uniciclo con l'obiettivo di implementare un osservatore di stato e un controllore predittivo (MPC) per muovere il sistema da uno stato iniziale a uno stato desiderato.
<img width="400" height="252" alt="image" src="https://github.com/user-attachments/assets/4d461ddc-d97c-4f51-8dbc-c2e07323846a" />

---
## Panoramica

Il progetto **ROBILAUT** riguarda la progettazione del controllo del moto di un robot mobile di tipo uniciclo. L'obiettivo è controllare un punto situato sull'asse di simmetria del robot, a una distanza specificata dal punto centrale tra le due ruote, portandolo alle coordinate desiderate ```(xd, yd)``` con orientamento ottimale.


## Obiettivo Progetto
Il robot deve muoversi da uno stato iniziale `x = [0, 0, 0]ᵀ` a uno stato desiderato definito come:

```math
x_d = 10, \quad y_d = 2, \quad \theta_d = \text{atan2}(y_d - y, x_d - x)
```

Il progetto comprende:
- Progettazione di un **Filtro di Kalman Esteso (EKF)** per l'osservazione dello stato
- Implementazione di un **Controllore Predittivo (MPC)** per ottenere il comportamento desiderato
- Confronto tra tre diverse strategie di controllo: LQR, MPC non vincolato, MPC vincolato
---

## Modello del Sistema
- **Coordinate del punto da controllare**: \(x, y\)
- **Orientamento**: $\theta$
- **Velocità**:
  - Lineare: $v$
  - Angolare: $\omega$
<img width="400" height="282" alt="image" src="https://github.com/user-attachments/assets/48d39850-65d2-4e2e-81db-e5f314e83ec7" />

### Variabili di Stato e Controllo

```math
\mathbf{x} = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}, \quad 
\mathbf{u} = \begin{bmatrix} v \\ \omega \end{bmatrix}
```
### Dinamica del Sistema
```math
\begin{aligned}
\dot{x} &= v \cos(\theta) - \delta \omega \sin(\theta) \\
\dot{y} &= v \sin(\theta) + \delta \omega \cos(\theta) \\
\dot{\theta} &= \omega
\end{aligned}
```
### Posizione del Punto Controllato
La posizione del punto controllato è descritta da:
```math
\begin{aligned}
x &= x_0 + \delta \cos(\theta) \\
y &= y_0 + \delta \sin(\theta)
\end{aligned}
```
### Sistema di Misura
Un sistema satellitare è montato sul veicolo e misura la posizione a una distanza $\alpha$ dal centro ($x_0, y_0$) lungo l’asse $y_b$:
```math
\begin{aligned}
x_m &= x_0 - \alpha \cos(\theta) \\
y_m &= y_0 + \alpha \sin(\theta)
\end{aligned}
```
Il sistema di misura fornisce anche $\theta$ con una frequenza di 10 Hz.

## Parametri del Sistema

| Parametro | Simbolo | Valore | Unità |
|-----------|---------|--------|-------|
| Distanza punto controllo | δ | 0.1 | m |
| Distanza sensore | α | 0.2 | m |
| Passo di campionamento | T | 0.1 | s |
| Velocità lineare max | v_max | 0.5 | m/s |
| Velocità angolare max | ω_max | 0.3 | rad/s |

## Implementazione

### 1. Modello in Spazio di Stato
- **Linearizzazione** del modello non lineare attorno al punto di equilibrio
- **Discretizzazione** con passo di campionamento T = 0.1 s usando il metodo di Eulero
- **Parametri**: δ = 0.1 m, α = 0.2 m

### 2. Strategie di Controllo Implementate

| Controllore | Descrizione | Vincoli | 
|-------------|-------------|---------|
| **LQR** | Linear Quadratic Regulator | Nessun vincolo |
| **MPC Senza Vincoli** | Model Predictive Control | Nessun vincolo |
| **MPC Con Vincoli** | Model Predictive Control | \|v\| < 0.5 m/s, \|ω\| < 0.3 rad/s |

### 3. Osservatore Stocastico - Filtro di Kalman Esteso

- **Inizializzazione**: 
  ```matlab
  Ps = 100·diag([0.01 0.01 1]), P = I₃
- **Covarianze**:
    ```matlab
    R_w = diag([0.1 0.1 0.5]), R_v = 0.5·diag([0.1 0.1 1])
- **Aggiornamento misura**:
  ```matlab
    K = Ps*C'*inv(R_v + C*Ps*C');
    x_hat = x_hats + K*(y + [-x_hats(1)+δ*cos(x_hats(3))+α*cos(x_hats(3)); ...]);
- **Aggiornamento temporale**: 
  ```matlab
    x_hats = x_hat + T*([u(1)*cos(x_hat(3))-δ*u(2)*sin(x_hat(3)); ...]);
    Ps = A*P*A' + R_w;
  
### Selezione modalità di controllo 
Il codice supporta tre modalità selezionabili tramite variabile d:
- d = 0: Controllore LQR
- d = 1: MPC non vincolato
- d = 2: MPC vincolato

## Risultati

### Prestazioni dei Controllori
- **Filtro di Kalman**: efficace soppressione del rumore di misura
 <img width="1758" height="876" alt="image" src="https://github.com/user-attachments/assets/caf919c6-6143-465a-9919-2f27069d31d8" />

- **MPC senza vincoli**: Performance ottimali ma senza limiti fisici
<img width="1758" height="890" alt="image" src="https://github.com/user-attachments/assets/f44dd2ce-d8a2-41d8-9454-f524a515b21e" />

- **MPC con vincoli**: Migliore prestazione in scenari realisti con limiti fisici
<img width="1250" height="826" alt="image" src="https://github.com/user-attachments/assets/a587f053-ff8f-48d5-a082-fade5af08ec0" />

- **LQR**: Prestazioni ottimali in assenza di vincoli
<img width="1788" height="893" alt="image" src="https://github.com/user-attachments/assets/6ffae760-8866-4a83-8f54-330d50748a7d" />


## Installazione ed Esecuzione

### Prerequisiti
- **MATLAB R2023b** o superiore
- **Optimization Toolbox**
- **Control System Toolbox**

### Esecuzione

```bash
git clone https://github.com/noemilatorre/Teoria_dei_Sistemi.git
```
```matlab
% Aprire il file principale in MATLAB
% Modificare la variabile 'd' per selezionare la modalità di controllo:
% d = 0 per LQR, d = 1 per MPC non vincolato, d = 2 per MPC vincolato
% Eseguire lo script
```

## Autore
**Noemi La Torre**

- Email: latorre.noemi17@gmail.com
- LinkedIn: [linkedin.com/in/noemilatorre](https://linkedin.com/in/noemilatorre)
- GitHub: [github.com/noemilatorre](https://github.com/noemilatorre)
- Portfolio: [noemilatorre.github.io](https://noemilatorre.github.io)

---

*Progetto sviluppato per il corso di Teoria dei Sistemi presso l'Università degli Studi di Cassino e del Lazio Meridionale.*



