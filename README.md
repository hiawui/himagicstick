## 卡尔曼滤波

$$
\begin{aligned}
\mathbf{predict} \\
{x_{k|k-1}} & = F_k \hat{x}_{k-1|k-1} + B_k u_k \\
P_{k|k-1} & = F_k P_{k-1|k-1} F_k^T + Q_k \\
\\
\mathbf{update} \\
\hat{y}_k & = z_k - H_k \hat{x}_{k|k-1} \\
S_k & = H_k P_{k|k-1} H_k^T + R_k \\
K_k & = P_{k|k-1} H_k^T S_k^{-1} \\
\\
\hat{x}_{k|k} & = \hat{x}_{k|k-1} + K_k \hat{y}_k \\
P_{k|k} & = (I - K_k H_k) P_{k|k-1} \\
\end{aligned}
\\ \begin{aligned} \\ \\ \end{aligned} \\
\begin{aligned}
\\
& x_k: k时刻, 重力相对芯片的单位向量 \\
& u_k: 角速度 \\
& q_k: dt * u_k \space 四元数旋转 \\
& F_k: q_k^{-1} \\
& x_k = F_k * x_{k-1} \\
\\
&P_k: x_k的协方差, 未知, 给随机初始值 \\
&Q: 角速度协方差 \\
\\ \\
&z_k: 加速度计加速度单位向量, 假设只有重力影响 \\
\end{aligned}
$$
