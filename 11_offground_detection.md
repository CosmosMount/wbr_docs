# 离地检测与控制

由 [vmc](04_vmc.md) 我们知道

```math
\left[ \begin{matrix}
F_{L} \\
\tau_{l}
\end{matrix} \right]=
(J^{T})^{-1}\left[ \begin{matrix}
\tau_{1} \\
\tau_{4}
\end{matrix} \right]=
(J^{T})^{-1}\left[ \begin{matrix}
k_{\tau}I_{q_1} \\
k_{\tau}I_{q_4}
\end{matrix} \right]
```

那么我们可以从电机反馈解得此时等效杆上的输出力、力矩值，进一步计算足端支持力，再通过足端支持力的大小判定机器人所处的状态。

对于牛顿-欧拉法分析

```math
-F_{w,i}^{v}+F_{N,i}-m_{w}g=m_{w}a_{w,i}^{v}\implies F_{N,i}=F_{w,i}^{v}+m_{w}(g+a_{w,i}^{v})
```

$`a_{b}^{v}`$ 可以由IMU测量与处理后得到，因此我们将原来的推导顺序反过来，同时考虑腿长变化，得到

```math
F_{w,i}^{v} =F_{l,i}^{v}+m_{l}(a_{l,i}^v+g)=m_{b}(a_{b}^{v}+g)+m_{l}(a_{l,i}^{v}+g)
```

```math
a_{w,i}^{v}=a_{b}^{v}-\frac{ \partial ^{2} }{ \partial t } (l_{i}\cos \theta_{l})=a_{b}^{v}+l_{i}\sin \theta_{l,i}\ddot{\theta}_{l,i}+l_{i}\cos \theta_{l,i}\dot{\theta}_{l,i}^{2}-\ddot{l}_{i}\cos\theta_{l,i}+2\dot{l}_{i}\sin \theta_{l,i}\dot{\theta}_{l,i}
```

```math
a_{l,i}^{v}=a_{b}^{v}-\frac{ \partial ^{2} }{ \partial t } (l_{b,i}\cos \theta_{l,i})=a_{b}^{v}+l_{b,i}\sin \theta_{l,i}\ddot{\theta}_{l,i}+l_{b,i}\cos \theta_{l,i}\dot{\theta}_{l,i}^{2}-\ddot{l}_{b,i}\cos\theta_{l,i}+2\dot{l}_{b,i}\sin \theta_{l,i}\dot{\theta}_{l,i}
```

这里，$`F_{w}^{v}`$ 的计算实际没有考虑腿伸长力与气弹簧力的影响，在 [vmc](04_vmc.md) 中，我们对气弹簧的力 $F_s$ 进行了估计，对气弹簧力进行补偿之后，理论上等价于无气弹簧模型。因此记实际竖直压力

```math
F_{v,i} = F_{w,i}^{v} + (F_{L,i}-F_{s,i})\cos\theta_{l,i}
```

为了方便计算，忽略 $`\dot{\theta}_{l,i}^{2}`$ 项和交叉项

```math
a_{w,i}^{v}=a_{b}^{v}+l_{i}\sin \theta_{l,i}\ddot{\theta}_{l,i}-\ddot{l}_{i}\cos\theta_{l,i}
```

```math
a_{l,i}^{v}=a_{b}^{v}+l_{b,i}\sin \theta_{l,i}\ddot{\theta}_{l,i}-\ddot{l}_{b,i}\cos\theta_{l,i}
```

```math
F_{w,i}^{v} =m_{b}(a_{b}^{v}+g)+m_{l}(a_{l,i}^{v}+g)=(m_{b}+m_{l})(a_{b}^{v}+g)+m_{l}l_{b,i}\sin \theta_{l,i}\ddot{\theta}_{l,i}
```

那么，由于 $`l_{b,i}=\mu l_{i}`$，我们可以化简得到
```math
F_{N,i}=(m_{b}+m_{l}+m_{w})(a_{b}^{v}+g)+(\mu m_{l}+m_{w})(l_{l}\sin \theta_{l,i}\ddot{\theta}_{l,i}+\ddot{l}_{i}\cos\theta_{l,i})+(F_{L,i}-F_{s,i})\cos\theta_{l,i}
```