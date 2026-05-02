# 磁力计校准算法原理 (9 参数模型)

本文档解释了基于椭球拟合的磁力计校准引擎 (`MagCalibrator`) 内部的数学原理和推导过程。

## 1. 磁场畸变模型

对于一个完美的磁力计（无误差），在恒定磁场中（如地球磁场），当传感器在三维空间中自由旋转时，其测量到的磁场矢量 $\mathbf{B}_{true}$ 将位于一个半径为 $R$ 的完美球面上。

然而，由于制造缺陷（增益不匹配、非正交性）和周围环境的磁性干扰（软铁和硬铁），实际测量的磁场矢量 $\mathbf{B}_{raw}$ 发生了畸变。我们可以用以下线性模型来描述这种畸变：

$$ \mathbf{B}_{raw} = \mathbf{M}_{distortion} \mathbf{B}_{true} + \mathbf{Offset} $$

- $\mathbf{B}_{raw}$: 3x1 原始磁场矢量
- $\mathbf{B}_{true}$: 3x1 真实磁场矢量
- $\mathbf{M}_{distortion}$: 3x3 畸变矩阵（包含了比例因子误差和非正交误差，以及软铁干扰）
- $\mathbf{Offset}$: 3x1 偏移向量（硬铁干扰，即恒定偏置）

## 2. 校正目标

我们的目标是找到一个反向变换，从 $\mathbf{B}_{raw}$ 中恢复出 $\mathbf{B}_{true}$：

$$ \mathbf{B}_{true} = \mathbf{M}_{correction} (\mathbf{B}_{raw} - \mathbf{Offset}) $$

其中，校正矩阵 $\mathbf{M}_{correction} = \mathbf{M}_{distortion}^{-1}$。

## 3. 椭球方程推导

由于 $\mathbf{B}_{true}$ 位于半径为 $R$ 的球面上，因此有：

$$ ||\mathbf{B}_{true}||^2 = \mathbf{B}_{true}^T \mathbf{B}_{true} = R^2 $$

将 $\mathbf{B}_{true}$ 的表达式代入上式：

$$ (\mathbf{B}_{raw} - \mathbf{Offset})^T \mathbf{M}_{correction}^T \mathbf{M}_{correction} (\mathbf{B}_{raw} - \mathbf{Offset}) = R^2 $$

我们定义一个对称正定矩阵 $\mathbf{A}$：

$$ \mathbf{A} = \frac{\mathbf{M}_{correction}^T \mathbf{M}_{correction}}{R^2} $$

则等式变为：

$$ (\mathbf{B}_{raw} - \mathbf{Offset})^T \mathbf{A} (\mathbf{B}_{raw} - \mathbf{Offset}) = 1 $$

展开这个二次型：

$$ \mathbf{B}_{raw}^T \mathbf{A} \mathbf{B}_{raw} - 2 (\mathbf{A} \mathbf{Offset})^T \mathbf{B}_{raw} + \mathbf{Offset}^T \mathbf{A} \mathbf{Offset} - 1 = 0 $$

这是一个三维空间中椭球的一般方程。如果我们设 $\mathbf{v} = \mathbf{B}_{raw} = [x, y, z]^T$，上述方程可以写为标量形式的二次曲面：

$$ ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + gx + hy + iz + j = 0 $$

## 4. 最小二乘法求解 (椭球拟合)

在代码实现中，我们将常数项分离，并设等号右边为 1：

$$ ax^2 + by^2 + cz^2 + 2dxy + 2exz + 2fyz + gx + hy + iz = 1 $$

这包含了 9 个未知参数（$a, b, c, d, e, f, g, h, i$）。对于给定的 $N$ 个数据点 $(x_k, y_k, z_k)$，我们可以构建一个 $N \times 9$ 的设计矩阵 $\mathbf{D}$，并求解超定线性方程组 $\mathbf{D} \mathbf{P} = \mathbf{1}$，其中 $\mathbf{P}$ 是包含 9 个参数的列向量，$\mathbf{1}$ 是元素全为 1 的列向量。

使用最小二乘法 (`np.linalg.lstsq`) 可以直接求出 $\mathbf{P}$。

## 5. 恢复校正参数

从拟合得到的 9 个参数中，我们可以重构出系数矩阵 $\mathbf{A}_{fit}$ 和一次项系数向量 $\mathbf{U}_{fit}$：

$$ \mathbf{A}_{fit} = \begin{bmatrix} a & d & e \\ d & b & f \\ e & f & c \end{bmatrix}, \quad \mathbf{U}_{fit} = \begin{bmatrix} g \\ h \\ i \end{bmatrix} $$

### 5.1 计算偏移量 $\mathbf{Offset}$

对比展开后的矩阵形式，我们发现一次项系数为 $-2\mathbf{A} \mathbf{Offset}$。因此，可以解出偏移量：

$$ \mathbf{Offset} = -0.5 \mathbf{A}_{fit}^{-1} \mathbf{U}_{fit} $$

### 5.2 计算真实系数矩阵 $\mathbf{A}$

由于我们在拟合时强制常数项为 1（实际上应该是 $1 - \mathbf{Offset}^T \mathbf{A} \mathbf{Offset}$），所以拟合出的 $\mathbf{A}_{fit}$ 需要进行缩放：

$$ \mathbf{A} = \mathbf{A}_{fit} \cdot \frac{1}{1 + \mathbf{Offset}^T \mathbf{A}_{fit} \mathbf{Offset}} $$

### 5.3 计算校正矩阵 $\mathbf{M}_{correction}$

根据之前的定义：

$$ \mathbf{M}_{correction}^T \mathbf{M}_{correction} = R^2 \mathbf{A} $$

设 $\mathbf{W} = R^2 \mathbf{A}$。因为 $\mathbf{W}$ 是对称正定矩阵，我们可以通过对 $\mathbf{W}$ 求矩阵平方根来得到对称的校正矩阵 $\mathbf{M}_{correction}$：

$$ \mathbf{M}_{correction} = \sqrt{\mathbf{W}} $$

在 Python 中，可以使用 `scipy.linalg.sqrtm` 来计算矩阵的平方根。

至此，我们成功从无序的畸变散点中提取出了硬铁偏移向量 $\mathbf{Offset}$ 和软铁/非正交校正矩阵 $\mathbf{M}_{correction}$，最终实现了对 AUV 磁力计的高精度校准。