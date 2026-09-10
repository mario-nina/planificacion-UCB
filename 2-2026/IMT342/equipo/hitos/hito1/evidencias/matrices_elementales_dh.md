# Matrices elementales D-H — ABB IRB 120

## Matriz general

La matriz D-H estándar utilizada es:

\[
{}^{i-1}T_i =
\begin{bmatrix}
\cos\theta_i & -\sin\theta_i\cos\alpha_i & \sin\theta_i\sin\alpha_i & a_i\cos\theta_i \\
\sin\theta_i & \cos\theta_i\cos\alpha_i & -\cos\theta_i\sin\alpha_i & a_i\sin\theta_i \\
0 & \sin\alpha_i & \cos\alpha_i & d_i \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

\[
c_i = \cos(q_i),
\qquad
s_i = \sin(q_i)
\]

## Transformaciones elementales

### \({}^{0}T_1\)

\[
{}^{0}T_1 =
\begin{bmatrix}
c_1 & 0 & -s_1 & 0 \\
s_1 & 0 & c_1 & 0 \\
0 & -1 & 0 & 0.290 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

### \({}^{1}T_2\)

\[
\theta_2 = q_2-\frac{\pi}{2}
\]

\[
{}^{1}T_2 =
\begin{bmatrix}
s_2 & c_2 & 0 & 0.270s_2 \\
-c_2 & s_2 & 0 & -0.270c_2 \\
0 & 0 & 1 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

### \({}^{2}T_3\)

\[
{}^{2}T_3 =
\begin{bmatrix}
c_3 & 0 & -s_3 & 0.070c_3 \\
s_3 & 0 & c_3 & 0.070s_3 \\
0 & -1 & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

### \({}^{3}T_4\)

\[
{}^{3}T_4 =
\begin{bmatrix}
c_4 & 0 & s_4 & 0 \\
s_4 & 0 & -c_4 & 0 \\
0 & 1 & 0 & 0.302 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

### \({}^{4}T_5\)

\[
{}^{4}T_5 =
\begin{bmatrix}
c_5 & 0 & -s_5 & 0 \\
s_5 & 0 & c_5 & 0 \\
0 & -1 & 0 & 0 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

### \({}^{5}T_6\)

\[
{}^{5}T_6 =
\begin{bmatrix}
c_6 & -s_6 & 0 & 0 \\
s_6 & c_6 & 0 & 0 \\
0 & 0 & 1 & 0.072 \\
0 & 0 & 0 & 1
\end{bmatrix}
\]

## Transformación total

\[
{}^{0}T_6 =
{}^{0}T_1
{}^{1}T_2
{}^{2}T_3
{}^{3}T_4
{}^{4}T_5
{}^{5}T_6
\]

## Parámetros D-H

| Junta | \(\theta_i\) | \(d_i\) [m] | \(a_i\) [m] | \(\alpha_i\) |
|:---:|:---:|:---:|:---:|:---:|
| J1 | \(q_1\) | 0.290 | 0.000 | \(-\frac{\pi}{2}\) |
| J2 | \(q_2-\frac{\pi}{2}\) | 0.000 | 0.270 | \(0\) |
| J3 | \(q_3\) | 0.000 | 0.070 | \(-\frac{\pi}{2}\) |
| J4 | \(q_4\) | 0.302 | 0.000 | \(\frac{\pi}{2}\) |
| J5 | \(q_5\) | 0.000 | 0.000 | \(-\frac{\pi}{2}\) |
| J6 | \(q_6\) | 0.072 | 0.000 | \(0\) |
