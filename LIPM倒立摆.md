# LIPM倒立摆

![image](https://github.com/user-attachments/assets/b8bf5f44-599b-4624-9757-fcc4be8973cf)


质心在力f的作用下上下移动，r为支撑点与质心的距离，f可以分为x,y,z三个方向的分量：

$$
\begin{align*}
f_x&=(x/r)f\\
f_y&=(y/r)f\\
f_z&=(z/r)f
\end{align*}
$$

$$
\begin{align*}
M\ddot{x}&=(x/r)f\\
M\ddot{y}&=(y/r)f\\
M\ddot{z}&=(z/r)f - Mg
\end{align*}
$$

定义约束面：

$$
z = k_xx + k_yy+z_c
$$

其中，kx,ky决定平面的倾斜，zc决定高度。

为了使质心在约束面上运动，其加速度应该保持与约束面的法向量垂直，即：

$$
\begin{bmatrix}
f(\frac{x}{r}), f(\frac{y}{r}), f(\frac{z}{r}) - Mg
\end{bmatrix}
\begin{bmatrix}
-k_x\\ -k_y\\ 1
\end{bmatrix}=0
$$

解得f：

$$
f = \frac{Mg}{z_c}
$$

因此，我们可以通过控制与腿长 r 成正比的伸缩力 f 来控制质心在约束面上运动。

可以求得质心在水平方向的运动方程如下：

$$
\begin{align*}
\ddot{x}&=\frac{g}{z_c}x\\
\ddot{y}&=\frac{g}{z_c}y
\end{align*}
$$

double_legs_test_fixed_switch_time.py

源码地址：https://github.com/chauby/BipedalWalkingRobots.git

轨道能量（OrbitalEnergy）：

$$
E = \frac{1}{2}v^2 - \frac{g}{2z}x^2
$$


①首先更新状态

```python
def update(self):
self.t += self.delta_t#时长
self.xt, self.vt = self.updateXtVt(self.x0, self.v0, self.t)#位置
self.zt = self.zc
self.orbital_energy = self.updateOrbitalEnergy(self.xt, self.vt)#速度
```

②根据时长更新质心位置和速度

```python
def updateXtVt(self, x0, v0, t):
    tau = t/self.Tc
    xt = x0*cosh(tau) + self.Tc*v0*sinh(tau)
    vt = (x0/self.Tc)*sinh(tau) + v0*cosh(tau)
    return xt, vt
```

③根据速度和位置更新轨道能量

```python
def updateOrbitalEnergy(self, x, v):
    orbital_energy = (v**2 - (self.g/(self.zc))*x**2)/2
    return orbital_energy
```

④根据速度和轨道能量计算捕获点

```python
def updateCapturePoint(self, xt, vt, target_OE):
    if(vt**2 < 2*target_OE):
        capture_point = self.capture_point
        print("warning: calculate the capture point failed, the velocity is too low. caputre_point=", capture_point)
    else:
        capture_point = xt + np.sign(vt)*sqrt((vt**2 - 2*target_OE)*self.zc/self.g)
    return capture_point
```

⑤根据捕获点计算落脚点

```python
#起步阶段
if LIPM_model.support_leg == 'left_leg':
    LIPM_model.right_foot_x = LIPM_model.left_foot_x + LIPM_model.capture_point
else:
    LIPM_model.left_foot_x = LIPM_model.right_foot_x + LIPM_model.capture_point
#周期性步态阶段
if LIPM_model.support_leg == 'left_leg':
    LIPM_model.right_foot_x = swing_foot_reference_x[j]#插值好的摆动腿轨迹
else:
    LIPM_model.left_foot_x = swing_foot_reference_x[j]
```

⑥根据仿真步长进行步态切换

```python
if((i >= switch_index) and (i % switch_index == 0)):#每走switch_index步，切换一次支撑腿
    LIPM_model.switchSupportLeg()
    # 计算位置、速度、轨道能量以及捕获点
    xt, vt = LIPM_model.updateXtVt(LIPM_model.x0, LIPM_model.v0, LIPM_model.delta_t*(switch_index))
    orbital_energy = LIPM_model.updateOrbitalEnergy(xt, vt)
    capture_point = LIPM_model.updateCapturePoint(xt, vt, LIPM_model.target_orbital_energy)
    # 根据支撑腿的不同，插值生成摆动腿的轨迹
    if LIPM_model.support_leg == 'left_leg':
        swing_foot_reference_x = np.linspace(LIPM_model.right_foot_x, LIPM_model.left_foot_x + capture_point, switch_index)
    else:
        swing_foot_reference_x = np.linspace(LIPM_model.left_foot_x, LIPM_model.right_foot_x + capture_point, switch_index)
    j = 0
```

⑦支撑腿切换，根据落脚点，更新x0 . x0继续用于更新xt、vt

```python
def switchSupportLeg(self):
    if self.support_leg == 'left_leg':
        self.x0 = self.left_foot_x + self.xt - self.right_foot_x
        self.support_leg = 'right_leg'
    elif self.support_leg == 'right_leg':
        self.x0 = self.right_foot_x + self.xt - self.left_foot_x
        self.support_leg = 'left_leg'
    else:
        print('Error: support leg is a wrong value:', self.support_leg)
```
![LIPM_double_legs_forward](https://github.com/user-attachments/assets/16466a37-1057-4b7e-8d6c-2414541042a1)


