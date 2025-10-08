# ROBILAUT - Controllo del Moto di un Robot Mobile

[![MATLAB](https://img.shields.io/badge/MATLAB-R2023b-orange?logo=mathworks)](https://www.mathworks.com/)
[![Control Systems](https://img.shields.io/badge/Control-MPC%20%26%20LQR-blue)](https://www.mathworks.com/products/control.html)
[![Status](https://img.shields.io/badge/Status-Completed-success)]()

Progetto di **Teoria dei Sistemi** per la progettazione del controllo del moto di un robot mobile di tipo uniciclo. Il sistema implementa strategie di controllo avanzate per il tracking di traiettorie con vincoli operativi.

---

## Panoramica

Il progetto **ROBILAUT** riguarda la progettazione del controllo del moto di un robot mobile di tipo uniciclo. L'obiettivo è controllare un punto situato sull'asse di simmetria del robot, a una distanza specificata dal punto centrale tra le due ruote, portandolo alle coordinate desiderate \((x_d, y_d)\) con orientamento ottimale.

**Obiettivo principale:** Portare il robot alle coordinate desiderate \((x_d, y_d)\) con orientamento \(\theta_d = \text{atan2}(y_d - y, x_d - x)\), partendo da condizioni iniziali \((x_0, y_0, \theta_0)\).

---

## Modello del Sistema
Il modello del robot considera:
- **Coordinate del punto da controllare**: \(x, y\)
- **Orientamento**: $\theta$
- **Velocità**:
  - Lineare: $v$
  - Angolare: $\omega$

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




## Implementazione

### 1. Modello in Spazio di Stato
- **Linearizzazione** del modello non lineare around del punto di equilibrio
- **Discretizzazione** con passo di campionamento T = 0.1 s
- **Parametri**: δ = 0.1 m, α = 0.2 m

### 2. Strategie di Controllo Implementate

| Controllore | Descrizione | Vincoli |
|-------------|-------------|---------|
| **LQR** | Linear Quadratic Regulator | Nessun vincolo |
| **MPC Senza Vincoli** | Model Predictive Control | Nessun vincolo |
| **MPC Con Vincoli** | Model Predictive Control | \|v\| < 0.5 m/s, \|ω\| < 0.3 rad/s |

### 3. Osservatore Stocastico
- **Filtro di Kalman** per la stima dello stato sulla base delle misure del sensore
- Gestione del rumore di misura del sensore satellitare
- Frequenza di aggiornamento: 10 Hz

## Risultati

### Prestazioni dei Controllori
- **MPC con vincoli**: Migliore prestazione in scenari realisti con limiti fisici
- **LQR**: Prestazioni ottimali in assenza di vincoli
- **Filtro di Kalman**: efficace soppressione del rumore di misura

### Metriche di Valutazione
- Tempo di assestamento
- Overshoot
- Rispetto dei vincoli operativi
- Robustezza al rumore di misura

## Parametri del Sistema

| Parametro | Simbolo | Valore | Unità |
|-----------|---------|--------|-------|
| Distanza punto controllo | δ | 0.1 | m |
| Distanza sensore | α | 0.2 | m |
| Passo di campionamento | T | 0.1 | s |
| Velocità lineare max | v_max | 0.5 | m/s |
| Velocità angolare max | ω_max | 0.3 | rad/s |

## Installazione ed Esecuzione

### Prerequisiti
- **MATLAB R2023b** o superiore
- **Optimization Toolbox**
- **Control System Toolbox**

### Esecuzione

```bash
git clone https://github.com/noemilatorre/Teoria_dei_Sistemi.git
#Esecuzione dello script matlab con FALG per i diversi casi
```

## Autore
**Noemi La Torre**

- Email: latorre.noemi17@gmail.com
- LinkedIn: [linkedin.com/in/noemilatorre](https://linkedin.com/in/noemilatorre)
- GitHub: [github.com/noemilatorre](https://github.com/noemilatorre)
- Portfolio: [noemilatorre.github.io](https://noemilatorre.github.io)

---

*Progetto sviluppato per il corso di Teoria dei Sistemi presso l'Università degli Studi di Cassino e del Lazio Meridionale.*



