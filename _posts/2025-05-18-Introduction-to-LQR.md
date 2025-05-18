---
title: 倒立摆仿真与控制
date: 2025-05-11
categories: [控制理论, 优化控制]
tags: [MPC, Automatic]
math: true
---

# 倒立摆仿真与控制

---

## 一. 物理建模

### 1. 数学物理模型

<img src="https://i-blog.csdnimg.cn/blog_migrate/4f7d81ca501bd62ddb92b79dc0c113a2.png" alt="a.png" style="zoom:40%;" />

---

### 2. 从拉格朗日方程说起

- **动能**  
  $$ 
  T = \underbrace{\frac{1}{2}M\dot{x}^2}_{\text{小车动能}} + \underbrace{\frac{1}{2}m\left(\dot{x}^2 + l^2\dot{\theta}^2 + 2l\dot{x}\dot{\theta}\cos\theta\right) + \frac{1}{2}\frac{1}{12}m(2l)^2\dot{\theta}^2}_{\text{杆的动能}}
  $$

- **总势能**  
  $$
  V = mgl\cos\theta 
  $$

- **拉格朗日量**  
  $$
  \mathcal{L}= T - V = \frac{1}{2}(M + m)\dot{x}^2 + \frac{2}{3}ml^2\dot{\theta}^2 + ml\dot{x}\dot{\theta}\cos\theta - mgl\cos\theta 
  $$

---

### 3. 运动方程推导

- **对 $x$ 的欧拉-拉格朗日方程**  
  $$
  \frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial\dot{x}}\right) - \frac{\partial\mathcal{L}}{\partial x} = F \\ 
  \Rightarrow (M + m)\ddot{x} + ml\ddot{\theta}\cos\theta - ml\dot{\theta}^2\sin\theta = F
  $$

- **对 $\theta$ 的欧拉-拉格朗日方程**  
  $$
  \frac{d}{dt}\left(\frac{\partial\mathcal{L}}{\partial\dot{\theta}}\right) - \frac{\partial\mathcal{L}}{\partial\theta} = 0 \\ 
  \Rightarrow \frac{4}{3}ml^2\ddot{\theta} + ml\ddot{x}\cos\theta - mgl\sin\theta = 0
  $$

---

### 4. 状态空间表达式（仿真器构建）

- **矩阵化表达**
  $$
  \begin{bmatrix} 
  M + m & m l \cos\theta \\ 
  m l \cos\theta & \frac{4}{3} m l^2 
  \end{bmatrix} 
  \begin{bmatrix} 
  \ddot{x} \\ 
  \ddot{\theta} 
  \end{bmatrix} 
  = 
  \begin{bmatrix} 
  u + m l \dot{\theta}^2 \sin\theta \\ 
  m g l \sin\theta 
  \end{bmatrix}
  $$
  
- **加速度表达式**
  $$
  \Delta = (M + m) \cdot \frac{4}{3} m l^2 - (m l \cos\theta)^2 
  $$
  
  $$
  \begin{bmatrix} 
  \ddot{x} \\ 
  \ddot{\theta} 
  \end{bmatrix} = 
  \frac{1}{\Delta} 
  \begin{bmatrix} 
  \frac{4}{3} m l^2 & -m l \cos\theta \\ 
  - m l \cos\theta & M + m 
  \end{bmatrix} 
  \begin{bmatrix} 
  u + m l \dot{\theta}^2 \sin\theta \\ 
  m g l \sin\theta 
  \end{bmatrix}
  $$

  展开得：
  $$
  \ddot{x} = \frac{ \frac{4}{3} m l^2 (u + m l \dot{\theta}^2 \sin\theta) - m^2 l^2 g \sin\theta \cos\theta }{ \Delta } 
  $$
  $$
  \ddot{\theta} = \frac{ -m l \cos\theta (u + m l \dot{\theta}^2 \sin\theta) + (M + m) m g l \sin\theta }{ \Delta }
  $$

- **状态空间变量定义**
  $$
  \begin{cases}
  x_1 = x \\
  x_2 = \dot{x} \\
  x_3 = \theta \\ 
  x_4 = \dot{\theta}
  \end{cases}
  $$
  $$
  \begin{cases} 
  \dot{x}_1 = x_2 \\ 
  \dot{x}_2 = \displaystyle \frac{ \frac{4}{3} m l^2 (u + m l x_4^2 \sin x_3) - m^2 l^2 g \sin x_3 \cos x_3 }{ (M + m) \cdot \frac{4}{3} m l^2 - (m l \cos x_3)^2 } \\ 
  \dot{x}_3 = x_4 \\ 
  \dot{x}_4 = \displaystyle \frac{ -m l \cos x_3 (u + m l x_4^2 \sin x_3) + (M + m) m g l \sin x_3 }{ (M + m) \cdot \frac{4}{3} m l^2 - (m l \cos x_3)^2 } 
  \end{cases}
  $$

---

## 二. 构建LQR控制器

### 1. 线性化状态空间表达式

- **小角度近似**  
  取 $\sin\theta \approx \theta$，$\cos\theta \approx 1$，$\dot{\theta}^2 \sin\theta \approx 0$：
  
  $$
  \begin{bmatrix} 
  M + m & m l \\ 
  m l & \frac{4}{3} m l^2 
  \end{bmatrix} 
  \begin{bmatrix} 
  \ddot{x} \\ 
  \ddot{\theta} 
  \end{bmatrix} 
  = \begin{bmatrix} 
  u \\ 
  m g l \theta 
  \end{bmatrix}
  $$
  
- **解出加速度表达式**
- 
  $$
  D = (M + m) \cdot \frac{4}{3} m l^2 - (m l)^2 = m l^2 \left( \frac{4}{3} M + \frac{1}{3} m \right) 
  $$
  
  $$
  \ddot{x} = \frac{ \frac{4}{3} m l^2 u - m^2 l^2 g \theta }{ D } 
  $$
  
  $$
  \ddot{\theta} = \frac{ -m l u + (M + m) m g l \theta }{ D }
  $$
  
- **状态空间表达式**
  $$
  \dot{\mathbf{x}} = A \mathbf{x} + B u 
  $$
  其中：
  $$
  \mathbf{x} = \begin{bmatrix} x \\ \dot{x} \\ \theta \\ \dot{\theta} \end{bmatrix}, \quad
  A = \begin{bmatrix} 
  0 & 1 & 0 & 0 \\ 
  0 & 0 & -\frac{m^2 g l^2}{D} & 0 \\ 
  0 & 0 & 0 & 1 \\ 
  0 & 0 & \frac{(M + m) m g l}{D} & 0 
  \end{bmatrix}, \quad
  B = \begin{bmatrix} 
  0 \\ 
  \frac{4}{3} m l^2 / D \\ 
  0 \\ 
  -m l / D 
  \end{bmatrix}
  $$

- **工程近似（$m \ll M$）**
  $$
  A = \begin{bmatrix} 
  0 & 1 & 0 & 0 \\ 
  0 & 0 & -\frac{m g}{M} & 0 \\ 
  0 & 0 & 0 & 1 \\ 
  0 & 0 & \frac{(M + m) g}{M l} & 0 
  \end{bmatrix}, \quad
  B = \begin{bmatrix} 
  0 \\ 
  \frac{1}{M} \\ 
  0 \\ 
  -\frac{1}{M l} 
  \end{bmatrix}
  $$

---

### 2. LQR控制器设计

- **性能指标**  
  $$
  J = \int_{0}^{\infty} \left( \mathbf{x}^T Q \mathbf{x} + u^T R u \right) dt
  $$

- **Riccati方程解**  
  $$
  A^T P + P A - P B R^{-1} B^T P + Q = 0
  $$

- **最优控制律**  
  $$
  u = -K \mathbf{x}, \quad K = R^{-1} B^T P
  $$

---

## 延伸阅读

- [一阶倒立摆建模与控制系统设计](https://blog.csdn.net/qq_42731705/article/details/122464642)
