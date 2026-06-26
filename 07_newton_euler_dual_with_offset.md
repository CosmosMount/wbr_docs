## 双腿质心偏移建模

将偏置并联构型的腿部质心靠后问题纳入物理建模中，通过对平衡点的显式计算得到了更精确的平衡位置与控制效果，由港大首次提出，是新兴的建模方法。同时将原先的参考质心改为参考转轴，简化了对腿部的分析，也是最符合质心偏移的参考方式。

这里的小角度线性化实际不发挥作用，准确的计算应当将角度带入到平衡点进行计算。精确的平衡点强依赖对模型的标注，标注不好可能最终得到的结果并不如人意。

### 轮转轴

左右轮对称，轮转轴与轮质心相同

#### 左轮

受力分析

```math
-F_{w,l}^{h}+f_{l}=m_{w}a_{w,l}^{h}
```
```math
-F_{w,l}^{v}+F_{N,l}-m_{w}g=a_{w,l}^{v}
```
加速度
```math
a_{w,l}^h=\ddot{x},a_{w,l}^{v}=0
```
转动分析
```math
I \ddot{\theta}_{w,l}=\tau_{w,l}-f_{l}R_{w}
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
I \ddot{\theta}_{w,r}=\tau_{w,r}-f_{r}R_{w}
```
#### 左右支持力

```math
F_{N,l}=F_{N,r}\implies F_{w,l}^{v}=F_{w,r}^{v}
```
### 腿转轴

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
a_{l,l}^{h}=a_{w,l}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{l}\sin \theta_{l,l})=\ddot{x}+l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}
```
```math
a_{l,l}^{v}=a_{w,l}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{l}\cos \theta_{l,l})=-l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}
```

转动分析
```math
I_{l,l}\ddot{\theta}_{l,l}=\tau_{l,l}-\tau_{w,l}-m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0})-F_{w,l}^{h}l_{l}\cos \theta_{l,l}+F_{w,l}^{v}l_{l}\sin \theta_{l,l}
```
#### 右腿

受力分析
```math
-F_{l,r}^{h}+F_{w,r}^{h}=m_{l}a_{l,r}^{h}
```
```math
-F_{l,r}^{v}+F_{w,r}^{v}-m_{l}g=m_{l}a_{l,r}^{v}
```
加速度
```math
a_{l,r}^{h}=a_{w,r}^{h}+\frac{ \partial ^{2} }{ \partial t } (l_{r}\sin \theta_{l,r})=\ddot{x}+l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
```math
a_{l,r}^{v}=a_{w,r}^{v}+\frac{ \partial ^{2} }{ \partial t } (l_{r}\cos \theta_{l,r})=-l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
转动分析
```math
I_{l,r}\ddot{\theta}_{l,r}=\tau_{l,r}-\tau_{w,r}-m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0})-F_{w,r}^{h}l_{r}\cos \theta_{l,r}+F_{w,r}^{v}l_{r}\sin \theta_{l,r}
```

### 机体转轴

受力分析
```math
F_{l,l}^{h}+F_{l,r}^{h}=m_{b}a_{b}^h
```
```math
F_{l,l}^{v}+F_{l,r}^{v}-m_{b}g=m_{b}a_{b}^{v}
```
加速度
```math
a_{b}^{h}=\frac{1}{2}(a_{l,l}^{h}+a_{l,r}^{h})=\ddot{x}+\frac{1}{2}l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+\frac{1}{2}l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
```math
a_{b}^{v}=\frac{1}{2}(a_{l,l}^{v}+a_{l,r}^{v})=-\frac{1}{2}l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-\frac{1}{2}l\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-\frac{1}{2}l_{l}\sin \theta_{l,r}\ddot{\theta}_{l,r}-\frac{1}{2}l\cos \theta_{l,r}\dot{\theta}_{l,r}^{2}
```
转动分析
```math
I_{b}\ddot{\theta}_{b}=-(\tau_{l,l}+\tau_{l,r})-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+(F_{l,l}^{v}+F_{l,r}^{v})d_{b}\sin \theta_{b}-(F_{l,l}^{h}+F_{l,r}^{h})d_{b}\cos \theta_{b}
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

#### 水平运动方程
```math
\begin{aligned}
f  &  = m_{w}a_{w,l}^{h}+m_{w}a_{w,r}^{h}+m_{l}a_{l}^{h}+m_{l}a_{l,r}^{h}+m_{b}a_{b}^{h}  \\
 & = m_{w}\ddot{x}+m_{w}\ddot{x}+m_{l}a_{l,l}^{h}+m_{l}a_{l,r}^{h}+m_{b} \frac{a_{l,l}^{h}+a_{l,r}^{h}}{2} \\
 & = (2m_{w}+2m_{l}+m_{b})\ddot{x} \\
  & +\frac{1}{2}(2m_{l}+m_{b})(l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}) \\
   & + \frac{1}{2}(2m_{l}+m_{b})(l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l}\dot{\theta}_{l,r}^{2})
\end{aligned}
```
代入 $`f`$
```math
\begin{aligned}
\frac{\tau_{w,l}+\tau_{w,r}}{R_{w}}  =& \left( 2m_{w}+2m_{l}+m_{b}+\frac{2I_{w}}{R_{w}^{2}} \right)\ddot{x} \\
  & +\frac{1}{2}(2m_{l}+m_{b})(l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}) \\
   & + \frac{1}{2}(2m_{l}+m_{b})(l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l}\dot{\theta}_{l,r}^{2})
\end{aligned}
```
#### yaw转动方程

```math
I_{\phi}\ddot{\phi}=\frac{R_{b}}{R_{w}}\Big[(\tau_{w,r}-I\ddot{\theta}_{w,r})-(\tau_{w,l}-I\ddot{\theta}_{w,l})\Big]
```
#### 机体转动方程

```math
\begin{aligned}
I_{b} \ddot{\theta}_{b} = & -(\tau_{l,l}+\tau_{l,r})-m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})+m_{b}gd_{b}\sin \theta_{b}-m_{b}\ddot{x}\cos \theta_{b} \\
 & +\frac{m_{b}}{2}(-l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}-l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2})d_{b}\sin \theta_{b} \\
 & -\frac{m_{b}}{2}(l_{l}\cos \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\sin \theta_{l,l}\dot{\theta}_{l,l}^{2}+l_{r}\cos \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\sin \theta_{l,r}\dot{\theta}_{l,r}^{2})\cos \theta_{b}
\end{aligned}
```

#### 腿部转动方程

```math
\begin{aligned}
F_{w,l}^{v} & =F_{l,l}^{v}+m_{l}(a_{l,l}^{v}+g)=m_{b}(a_{b}^{v}+g)+m_{l}(a_{l,l}^{v}+g) = \frac{1}{2}(m_{b}+2m_{l})a_{l,l}^{v}+\frac{1}{2}m_{b}a_{l,r}^{v} \\
 & =\frac{1}{2}(m_{b}+2m_{l})(-l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2})+\frac{1}{2}m_{b}(-l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2})
\end{aligned}
```
```math
\begin{aligned}
I_{l,l}\ddot{\theta}_{l,l}= & \tau_{l,l}-\tau_{w,l}-m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0}) \\
 & -l_{w,l}\cos \theta_{l,l}(\frac{\tau_{w,l}-I_{w} \ddot{\theta}_{w,l}}{R_{w}}-m_{w}\ddot{x}) \\
 & +\frac{1}{2}(m_{b}+2m_{l})l_{r}\sin \theta_{l,l}(-l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}) \\
& +\frac{1}{2}m_{b}l_{r}\sin \theta_{l,l}(-l_{r}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{r}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2})
\end{aligned}
```

```math
\begin{aligned}
I_{l,r}\ddot{\theta}_{l,r}= & \tau_{l,r}-\tau_{w,r}-m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0}) \\
 & -l_{w,r}\cos \theta_{l,r}(\frac{\tau_{w,r}-I_{w} \ddot{\theta}_{w,r}}{R_{w}}-m_{w}\ddot{x}) \\
 & +\frac{1}{2}(m_{b}+2m_{l})l_{r}\sin \theta_{l,r}(-l_{l}\sin \theta_{l,l}\ddot{\theta}_{l,l}-l_{l}\cos \theta_{l,l}\dot{\theta}_{l,l}^{2}) \\
& +\frac{1}{2}m_{b}l_{r}\sin \theta_{l,r}(-l_{r}\sin \theta_{l,r}\ddot{\theta}_{l,r}-l_{r}\cos \theta_{l,r}\dot{\theta}_{l,r}^{2})
\end{aligned}
```
### 标准动力学

```math
M(q)\ddot{q}+C(q,\dot{q})\dot{q}+G(q)=B(q)\tau
```

$`C`$ 汇集 $`\dot{\theta}^{2}`$ 离心项，此处不展开 $`M,B`$ 元素，只给维数与 $`G`$ 。

$`q=[\theta_{w,l},\theta_{w,r},\theta_{l,l},\theta_{l,r},\theta_{b}]^{T}`$ ， $`\tau=[\tau_{w,l},\tau_{w,r},\tau_{l,l},\tau_{l,r}]^{T}`$ 。$`M`$： $`5\times 5`$ ；$`B`$： $`5\times 4`$

```math
G=\begin{bmatrix}
0 \\
0 \\
\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{l}\sin\theta_{l,l}+m_{l}gd_{l}\sin(\theta_{l,l}+\theta_{l,l}^{0}) \\
\big(m_{l}+\tfrac{m_{b}}{2}\big)gl_{r}\sin\theta_{l,r}+m_{l}gd_{l}\sin(\theta_{l,r}+\theta_{l,r}^{0}) \\
m_{b}gd_{b}\cos(\theta_{b}+\theta_{b}^{0})-m_{b}gd_{b}\sin\theta_{b}
\end{bmatrix}
```

#### 平衡点计算

对于 $`q`$ 在平衡点，满足 $`G(q)=0`$ ，解得

```math
\theta_{l,l}^{\text{eq}} = \arctan\left( \dfrac{-m_l d_l \sin\theta_{l,l}^0}{\left(m_l + \frac{m_b}{2}\right) l_l + m_l d_l \cos\theta_{l,l}^0} \right)
```

```math
\theta_{l,r}^{\text{eq}} = \arctan\left( \dfrac{-m_l d_l \sin\theta_{l,r}^0}{\left(m_l + \frac{m_b}{2}\right) l_r + m_l d_l \cos\theta_{l,r}^0} \right)
```

```math
\theta_{b} = \dfrac{\pi}{4} - \dfrac{\theta_{b}^0}{2}
```

测算之后将平衡点代入式子，计算得到 $`A,B`$ 矩阵