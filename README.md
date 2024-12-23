# ROBILAUT - Controllo del Moto di un Robot Mobile

## Descrizione del Progetto
Il progetto **ROBILAUT** riguarda la progettazione del controllo del moto di un robot mobile di tipo uniciclo. Lo scopo è controllare un punto situato sull’asse di simmetria del robot, a una distanza specificata dal punto centrale tra le due ruote.


### Modello del Sistema
Il modello del robot considera:
- **Coordinate del punto da controllare**: \(x, y\)
- **Orientamento**: $\theta$
- **Velocità**:
  - Lineare: $v$
  - Angolare: $\omega$

La posizione del punto controllato è descritta da:
\[
\begin{aligned}
x &= x_0 + \delta \cos(\theta) \\
y &= y_0 + \delta \sin(\theta)
\end{aligned}
\]

La dinamica del sistema è:
\[
\begin{aligned}
\dot{x} &= v \cos(\theta) - \delta \omega \sin(\theta) \\
\dot{y} &= v \sin(\theta) + \delta \omega \cos(\theta) \\
\dot{\theta} &= \omega
\end{aligned}
\]

Le variabili di stato sono definite come:
\[
x = \begin{bmatrix} x \\ y \\ \theta \end{bmatrix}, \quad u = \begin{bmatrix} v \\ \omega \end{bmatrix}.
\]

### Sistema di Misura
Un sistema satellitare è montato sul veicolo e misura la posizione a una distanza $\alpha$ dal centro ($x_0, y_0$) lungo l’asse $y_b$:
\[
\begin{aligned}
x_m &= x_0 - \alpha \cos(\theta) \\
y_m &= y_0 + \alpha \sin(\theta)
\end{aligned}
\]
Il sistema di misura fornisce anche $\theta$ con una frequenza di 10 Hz.

### Obiettivo
Portare il robot alle coordinate desiderate $(x_d, y_d)$ con orientamento $\theta_d = \text{atan2}(y_d - y, x_d - x)$, partendo da condizioni iniziali $(x_0, y_0, \theta_0)$.

## Implementazione
### 1. Modello in Spazio di Stato
- Le equazioni di uscita sono state definite assumendo $\delta = 0.1$ m e $\alpha = 0.2$ m.
- Il modello è stato linearizzato e discretizzato con un passo di campionamento $T = 0.1$ s.

### 2. Controllore
Un **controllore a modello predittivo (MPC)** è stato progettato, considerando i seguenti vincoli:
- $|v| < 0.5 \, \text{m/s}$
- $|\omega| < 0.3 \, \text{rad/s}$

Tre approcci sono stati implementati:
1. **LQR** (Linear Quadratic Regulator) per un controllo senza vincoli.
2. **MPC senza vincoli**.
3. **MPC con vincoli**.

### 3. Osservatore Stocastico
Un filtro di Kalman è stato implementato per stimare lo stato del sistema basandosi sulle misure rumorose del sensore.


### Risultati
I risultati mostrano:
- Miglior precisione del MPC rispetto al LQR in presenza di vincoli.
- Capacità dell'osservatore stocastico di gestire il rumore nelle misure.

## Parametri Principali
| Parametro        | Valore  |
|-------------------|---------|
| \(\delta\)       | 0.1 m   |
| \(\alpha\)       | 0.2 m   |
| Passo \(T\)      | 0.1 s   |
| Vincoli \(v\)    | \([-0.5, 0.5]\) m/s |
| Vincoli \(\omega\) | \([-0.3, 0.3]\) rad/s |

## Requisiti
- MATLAB con toolbox di ottimizzazione.
- Familiarità con sistemi di controllo e teoria dei sistemi.


