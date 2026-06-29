## 双腿建模

一切力以竖直向上、水平向左为正方向

| 物理意义      | 数学符号                                            |
| --------- | ----------------------------------------------- |
| 轮主动力矩     | $`\tau_{w,l},\tau_{w,r}`$                         |
| 髋主动力矩     | $`\tau_{l,l},\tau_{l,r}`$                         |
| 腿对机身水平作用力 | $`F_{l,l}^{h},F_{l,r}^{h}`$                         |
| 腿对机身垂直作用力 | $`F_{l,l}^{v},F_{l,r}^{v}`$                         |
| 轮对腿水平作用力  | $`F_{w,l}^{h},F_{w,r}^{h}`$                         |
| 轮对腿垂直作用力  | $`F_{w,l}^{v},F_{w,r}^{v}`$                         |
| 轮质量与惯量    | $`m_{w},I_{w}`$                                   |
| 腿质量与惯量    | $`m_{l},I_{l}`$                                   |
| 机体质量与惯量   | $`m_{b},I_{b}`$                                   |
| 位移相关      | $`x,\dot{x},\ddot{x}`$                            |
| yaw       | $`\phi,\dot{\phi}, \ddot{\phi}`$                  |
| pitch     | $`\theta_{b},\dot{\theta}_{b},\ddot{\theta}_{b}`$ |
| 摆角        | $`\theta_{l},\dot{\theta}_{l},\ddot{\theta}_{l}`$ |
| 轮角        | $`\theta_{w},\dot{\theta}_{w},\ddot{\theta}_{w}`$ |
| 轮转轴到腿转轴距离 | $`l,l_{l},l_{r}`$                                 |
| 轮转轴到腿质心距离 | $`l_{w},l_{w,l},l_{w,r}`$                         |
| 腿转轴到腿质心距离 | $`l_{b},l_{b,l},l_{b,r}`$                         |
| 轮加速度      | $`a_{w}^{h},a_{w}^{v}`$                               |
| 腿加速度      | $`a_{l}^{h},a_{l}^{v}`$                               |
| 机体加速度     | $`a_{b}^{h},a_{b}^{v}`$                               |
| 机体质心偏移    | $`d_{b},\theta_{b}^{0}`$                            |
| 腿质心偏移     | $`d_{l},\theta_{l}^{0}`$                            |

将双腿分开建模，并将yaw实际作为状态变量进行计算，能够对双腿实现更精细的控制，并消除了调节劈叉环的过程，由上交首次提出，和单腿建模均为当前最主流建模。

### 轮质心

左右轮对称

#### 左轮

受力分析

```math
-F_{w,l}^{h}+f_{l}=m_{w}a_{w,l}^{h}
```
```math
-F_{w,l}^{v}+F_{N,l}-m_{w}g=m_{w}a_{w,l}^{v}
```
加速度

```math
a_{w,l}^h=\ddot{x},a_{w,l}^{v}=0
```
转动分析
```math
I_{w} \ddot{\theta}_{w,l}=\tau_{w,l}-f_{l}R_{w}
```

#### 右轮

```math
-F_{w,r}^{h}+f_{l}=m_{w}a_{w,r}^{h}
```
```math
-F_{w,r}^{v}+F_{N,r}-m_{w}g=m_{w}a_{w,r}^{v}
```
加速度

```math
a_{w,r}^h=\ddot{x},a_{w,r}^{v}=0
```
转动分析
```math
I_{w} \ddot{\theta}_{w,r}=\tau_{w,r}-f_{r}R_{w}
```
#### 左右支持力

```math
F_{N,l}=F_{N,r}\implies F_{w,l}^{v}=F_{w,r}^{v}
```
### 腿质心

左右腿对称

#### 左腿

受力分析
```math
-F_{l,l}^{h}+F_{w,l}^{h}=m_{l}a_{l,l}^{h}
```
```math
-F_{l,l}^{v}+F_{w,l}^{v}-m_{l}g=m_{l}a_{l,l}^{v}
```
加速度
```math
a_{l,l}^{h}=a_{w,l}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{w,l}\sin \theta_{l,l})=\ddot{x}+l_{w,l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}
```
```math
a_{l,l}^{v}=a_{w,l}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w,l}\cos \theta_{l,l})=-l_{w,l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}
```
转动分析
```math
I_{l,l}\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}+(F_{w,l}^{v}l_{w,l}+F_{l,l}^{v}l_{b,l})\sin \theta_{l,l}-(F_{w,l}^{h}l_{w,l}+F_{l,l}^{h}l_{b,l})\cos \theta_{l,l}
```
#### 右腿
```math
-F_{l,r}^{h}+F_{w,r}^{h}=m_{l}a_{l,r}^{h}
```
```math
-F_{l,r}^{v}+F_{w,r}^{v}-m_{l}g=m_{l}a_{l,r}^{v}
```
加速度
```math
a_{l,r}^{h}=a_{w,r}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{w,r}\sin \theta_{l,r})=\ddot{x}+l_{w,r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
```math
a_{l,r}^{v}=a_{w,r}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{w,r}\cos \theta_{l,r})=-l_{w,r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
转动分析
```math
I_{l,r}\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}+(F_{w,r}^{v}l_{w,r}+F_{l,r}^{v}l_{b,r})\sin \theta_{l,r}-(F_{w,r}^{h}l_{w,r}+F_{l,r}^{h}l_{b,r})\cos \theta_{l,r}
```
### 机体质心

```math
F_{l,l}^{h}+F_{l,r}^{h}=m_{b}a_{b}^{h}
```
```math
F_{l,l}^{v}+F_{l,r}^{v}-m_{b}g=m_{b}a_{b}^{v}
```
加速度

```math
\begin{aligned}
a_{b}^{h} & =a_{w}^{h}+\frac{ \partial ^2 }{ \partial t } \frac{1}{2}(l_{l}\sin \theta_{l,l}+l_{r}\sin \theta_{l,r}) \\
 & =\ddot{x}+\frac{1}{2}l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{1}{2}l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
\end{aligned}
```
```math
\begin{aligned}
a_{b}^{v} & =a_{w}^{v}+\frac{ \partial ^{2} }{ \partial t } \frac{1}{2}(l_{l} \cos \theta_{l,l}+l_{r}\cos \theta_{l,r}) \\
 & =-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
\end{aligned}
```
转动分析
```math
I_{b}\ddot{\theta}_{b}=-(\tau_{l,l}+\tau_{l,r})+(F_{l,l}^{v}+F_{l,r}^{v})d_{b}\sin \theta_{b}-(F_{l,l}^{h}+F_{l,r}^{h})d_{b}\cos \theta_{b}
```
```math
I_{\phi}\ddot{\phi }=(f_{r}-f_{l})R_{b}
```
### 控制律

我们需要
```math
x=\left[ \begin{matrix}
x \\
\dot{x}  \\
\phi \\
\dot{\phi} \\
\theta_{l,l} \\
\dot{\theta}_{l,l} \\
\theta_{l,r} \\
\dot{\theta}_{l,r} \\
\theta_{b} \\
\dot{\theta}_{b}
\end{matrix}
 \right]
 ,
 \dot{x}=\left[ \begin{matrix}
\dot{x} \\
\ddot{x}  \\
\dot{\phi} \\
\ddot{\phi} \\
\dot{\theta}_{l,l} \\
\ddot{\theta}_{l,l} \\
\dot{\theta}_{l,r} \\
\ddot{\theta}_{l,r} \\
\dot{\theta}_{b} \\
\ddot{\theta}_{b}
\end{matrix}
 \right]
 ,
  u=\left[ \begin{matrix}
\tau_{w,l}  \\
\tau_{w,r} \\
\tau_{l,l} \\
\tau_{l,r}
\end{matrix}
 \right]
```
最终期望整理出
```math
\dot{x}=Ax+Bu
```
消去力

#### 水平转动方程

```math
f  =m_{w,l}a_{w,l}^{h}+m_{w,r}a_{w,r}^{h}+m_{l,l}a_{l,l}^{h}+m_{l,r}a_{l,r}^{h}+m_{b}a_{b}^{h} 
```
代入 $`f`$
```math
\begin{aligned}
\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}} = &  \left( 2m_{w}+2m_{l}+m_{b}+\frac{2I_{w}}{R_{w}^{2}} \right)\ddot{x}\\
 & +\left( m_{l}+\frac{m_{b}}{2} \right)(l_{w,l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{w,r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}) \\
 & +\frac{m_{b}}{2}(l_{b,l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{b,l}\sin \theta_{l,l}\dot{\theta}_{l,l}^2+l_{b,r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{b,r}\sin \theta_{l,r}\dot{\theta}_{l,r}^2)
\end{aligned}
```


#### yaw转动方程

```math
I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]
```

#### 机体转动方程

```math
\begin{aligned}
I_{b} \ddot{\theta}_{b} = & -(\tau_{l,l}+\tau_{l,r})+m_{b}gd_{b}\sin \theta_{b}-m_{b}\ddot{x}\cos \theta_{b} \\
 & +\frac{m_{b}}{2}(-l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2})d_{b}\sin \theta_{b} \\
 & -\frac{m_{b}}{2}(l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2})\cos \theta_{b}
\end{aligned}
```

#### 腿部转动方程

```math
\begin{aligned}
F_{w,l}^{v} & =F_{l,l}^{v}+m_{l}(a_{l,l}^v+g)=m_{b}(a_{b}^{v}+g)+m_{l}(a_{l,l}^{v}+g) \\
 & = \frac{1}{2}m_{b}\left( -\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}+g \right)+m_{l}(-l_{w,l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\cos \theta_{l}\dot{\theta}_{l}^{2}+g)
\end{aligned}
```

```math
\begin{aligned}
I_{l,l} \ddot{\theta}_{l,l} = & \tau_{l,l}-\tau_{w,l} \\
 & +\frac{1}{2}m_{b}l_{l}\sin \theta_{l,l}(-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}+g) \\
 & + \frac{1}{2}m_{l}l_{w,l}\sin \theta_{l,l}(-l_{w,l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{w,l}\cos \theta_{l}\dot{\theta}_{l}^{2}+g) \\
 & -l_{w,l}\cos \theta_{l,l}(\frac{\tau_{w,l}-I_{w} \ddot{\theta}_{w,l}}{R_{w}}-m_{w}\ddot{x}) \\
 & -m_{b}l_{b,l}\cos \theta_{l,l}(\ddot{x}+l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2})
\end{aligned}
```

```math
\begin{aligned}
I_{l,r} \ddot{\theta}_{l,r} = & \tau_{l,r}-\tau_{w,r} \\
 & +\frac{1}{2}m_{b}l_{r}\sin \theta_{l,r}(-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{l}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}+g) \\
 & + \frac{1}{2}m_{l}l_{w,r}\sin \theta_{l,r}(-l_{w,r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{w,r}\cos \theta_{l}\dot{\theta}_{l}^{2}+g) \\
 & -l_{w,r}\cos \theta_{l,r}(\frac{\tau_{w,r}-I_{w} \ddot{\theta}_{w,r}}{R_{w}}-m_{w}\ddot{x}) \\
 & -m_{b}l_{b,r}\cos \theta_{l,r}(\ddot{x}+l_{l}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{l}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2})
\end{aligned}
```
