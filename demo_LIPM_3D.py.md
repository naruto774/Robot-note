demo_LIPM_3D.py

这个demo相较于使用轨道能量和捕获点的demo更加复杂。

一、定义变量

```python
COM_pos_x = list()          #质心的绝对位置
COM_pos_y = list()
left_foot_pos_x = list()    #左脚的绝对位置轨迹
left_foot_pos_y = list()
left_foot_pos_z = list()
COM_pos_0 = [-0.4, 0.2, 1.0]
COM_v0 = [1.0, -0.01]
left_foot_pos = [-0.2, 0.3, 0]#初始位置
right_foot_pos = [0.2, -0.3, 0]
s_x = 0.5       #步长在x轴方向的长度
s_y = 0.4       #步长在y轴方向的长度
a = 1.0         #调整步态的比例因子
b = 1.0
theta = 0.0     #步态的转角
self.p_x = 0  # 期望的足部位置x
self.p_y = 0  # desired foot location y        
self.x_0 = 0#	 COM initial state
self.vx_0 = 0
self.y_0 = 0
self.vy_0 = 0					
self.x_t = 0	# COM real-time state
self.vx_t = 0
self.y_t = 0
self.vy_t = 0
```

二、LIPM_model 参数赋值

```python
LIPM_model.support_leg = 'left_leg' # set the support leg to right leg in next step
if LIPM_model.support_leg == 'left_leg':
    support_foot_pos = LIPM_model.left_foot_pos
    LIPM_model.p_x = LIPM_model.left_foot_pos[0] 
    LIPM_model.p_y = LIPM_model.left_foot_pos[1]
else:
    support_foot_pos = LIPM_model.right_foot_pos
    LIPM_model.p_x = LIPM_model.right_foot_pos[0]
    LIPM_model.p_y = LIPM_model.right_foot_pos[1]
#x0=质心位置-支撑腿位置
LIPM_model.x_0 = LIPM_model.COM_pos[0] - support_foot_pos[0]#-0.4-（-0.2）=-0.2
LIPM_model.y_0 = LIPM_model.COM_pos[1] - support_foot_pos[1]
LIPM_model.vx_0 = COM_v0[0]
LIPM_model.vy_0 = COM_v0[1]
```

三、for循环

①LIPM_model.step()；用x0和v0求xt和vt

$$
\begin{cases}
x_t &= x_0 \cosh(t/T_c) + T_c \dot{x}_0 \sinh(t/T_c) \\
\dot{x}_t &= x_0/T_c \sinh(t/T_c) + \dot{x}_0 \cosh(t/T_c) \\
T_c &= \sqrt{z/g}
\end{cases}
$$


```python
def step(self):
    self.t += self.dt
    t = self.t
    T_c = self.T_c

    self.x_t = self.x_0*np.cosh(t/T_c) + T_c*self.vx_0*np.sinh(t/T_c)
    self.vx_t = self.x_0/T_c*np.sinh(t/T_c) + self.vx_0*np.cosh(t/T_c)

    self.y_t = self.y_0*np.cosh(t/T_c) + T_c*self.vy_0*np.sinh(t/T_c)
    self.vy_t = self.y_0/T_c*np.sinh(t/T_c) + self.vy_0*np.cosh(t/T_c)
```

②每在25个时间步后支撑腿交换，step_num+1，

```python
if (i > 0) and (i % switch_index == 0):
    j = 0

    LIPM_model.switchSupportLeg() # 交换支撑腿（质心位置）
    step_num += 1
```

```python
def switchSupportLeg(self):
    if self.support_leg is 'left_leg':
        print('\n---- switch the support leg to the right leg')
        self.support_leg = 'right_leg'
        COM_pos_x = self.x_t + self.left_foot_pos[0]
        COM_pos_y = self.y_t + self.left_foot_pos[1]
        self.x_0 = COM_pos_x - self.right_foot_pos[0]#质心位置-新支撑腿位置
        self.y_0 = COM_pos_y - self.right_foot_pos[1]
```

step_num每+1，要进行摆动腿位置计算；

step_num>=5，将步长设置为0；

step_num>+10，将横向步距设置为0。

③更新LIPM_model.p_x

```python
if LIPM_model.support_leg == 'left_leg':
    support_foot_pos = LIPM_model.left_foot_pos
    LIPM_model.p_x = LIPM_model.left_foot_pos[0]
    LIPM_model.p_y = LIPM_model.left_foot_pos[1]
else:
    support_foot_pos = LIPM_model.right_foot_pos
    LIPM_model.p_x = LIPM_model.right_foot_pos[0]
    LIPM_model.p_y = LIPM_model.right_foot_pos[1]
```

④使用x0和v0计算xt和vt，这一步和上面的①是一样的。

```python
def calculateXtVt(self, t):
    T_c = self.T_c

    x_t = self.x_0*np.cosh(t/T_c) + T_c*self.vx_0*np.sinh(t/T_c)
    vx_t = self.x_0/T_c*np.sinh(t/T_c) + self.vx_0*np.cosh(t/T_c)

    y_t = self.y_0*np.cosh(t/T_c) + T_c*self.vy_0*np.sinh(t/T_c)
    vy_t = self.y_0/T_c*np.sinh(t/T_c) + self.vy_0*np.cosh(t/T_c)

    return x_t, vx_t, y_t, vy_t
```

⑤更新x0

```python
if LIPM_model.support_leg == 'left_leg':
    x_0 = x_0 + LIPM_model.left_foot_pos[0] # need the absolute position for next step
    y_0 = y_0 + LIPM_model.left_foot_pos[1] # need the absolute position for next step
else:
    x_0 = x_0 + LIPM_model.right_foot_pos[0] # need the absolute position for next step
    y_0 = y_0 + LIPM_model.right_foot_pos[1] # need the absolute position for next step
```

⑥计算下一步的位置

```python
def calculateFootLocationForNextStep(self, s_x=0.0, s_y=0.0, a=1.0, b=1.0, theta=0.0, x_0=0.0, vx_0=0.0, y_0=0.0, vy_0=0.0):
    self.s_x = s_x
    self.s_y = s_y
    # 计算参考落脚点
    print(self.p_x, self.p_y)
    p_x_new, p_y_new = self.nextReferenceFootLocation(s_x, s_y, theta)
    # calculate desired COM states
    bar_x, bar_y = self.nextState(s_x, s_y, theta)
    bar_vx, bar_vy = self.nextVel(bar_x, bar_y, theta)
    # calculate target COM state in the next step
    self.x_d, self.vx_d = self.targetState(p_x_new, bar_x, bar_vx)
    self.y_d, self.vy_d = self.targetState(p_y_new, bar_y, bar_vy)
    # 修正落脚点，用到了ab
    self.p_x_star = self.modifiedFootLocation(a, b, self.x_d, self.vx_d, x_0, vx_0)
    self.p_y_star = self.modifiedFootLocation(a, b, self.y_d, self.vy_d, y_0, vy_0)
```

这里的nextReferenceFootLocation函数如下：用步态参数计算参考轨迹

```python
def nextReferenceFootLocation(self, s_x, s_y, theta=0):
    if self.support_leg is 'left_leg':
        p_x_new = self.p_x + np.cos(theta)*s_x - np.sin(theta)*s_y
        p_y_new = self.p_y + np.sin(theta)*s_x + np.cos(theta)*s_y
    elif self.support_leg is 'right_leg':
        p_x_new = self.p_x + np.cos(theta)*s_x + np.sin(theta)*s_y
        p_y_new = self.p_y + np.sin(theta)*s_x - np.cos(theta)*s_y

    return p_x_new, p_y_new
```

⑦拟合摆动腿位置

```python
if LIPM_model.support_leg == 'left_leg':
    right_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]
    swing_foot_pos[:,0] = np.linspace(LIPM_model.right_foot_pos[0], right_foot_target_pos[0], swing_data_len)
    swing_foot_pos[:,1] = np.linspace(LIPM_model.right_foot_pos[1], right_foot_target_pos[1], swing_data_len)
    swing_foot_pos[1:swing_data_len-1, 2] = 0.1
else:
    left_foot_target_pos = [LIPM_model.p_x_star, LIPM_model.p_y_star, 0]
    swing_foot_pos[:,0] = np.linspace(LIPM_model.left_foot_pos[0], left_foot_target_pos[0], swing_data_len)
    swing_foot_pos[:,1] = np.linspace(LIPM_model.left_foot_pos[1], left_foot_target_pos[1], swing_data_len)
    swing_foot_pos[1:swing_data_len-1, 2] = 0.1
```

一次for循环结束，进行第二次循环

⑧第二次循环step_num >= 1，可以生成摆动腿位置了

```python
LIPM_model.step()

if step_num >= 1:
    if LIPM_model.support_leg == 'left_leg':
        LIPM_model.right_foot_pos = [swing_foot_pos[j,0], swing_foot_pos[j,1], swing_foot_pos[j,2]]
    else:
        LIPM_model.left_foot_pos = [swing_foot_pos[j,0], swing_foot_pos[j,1], swing_foot_pos[j,2]]
    j += 1
```

如果当前时间步不是25的倍数就只执行⑧。



