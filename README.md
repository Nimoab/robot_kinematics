# Robot kinematics

## Notations

$$
\begin{array}{|c|l|}
\hline
   m &
   \textsf{The task space size} \\\hline
   n &
   \textsf{The configuration space size} \\\hline
   \bm l &
   \textsf{Column vector of size } n \textsf{ which component } l_i \textsf{ is the length of the link connecting } \\ & \textsf{the joint } i \textsf{ to the joint } i + 1\\\hline
   \bm x & 
   \textsf{End effector position in cartesian coordinates as an } m \textsf{ lines column vector}  \\\hline
   \bm q & 
   \textsf{An } n \textsf{ lines column vector of the configuration of the robot}  \\\hline
   \bm J & 
   \textsf{The } m\times n  \textsf{ Jacobian matrix}  \\\hline
   \dot{\bm x} & \textsf{Velocity of the end effector in task space}\\\hline
   \dot{\bm q} & \textsf{Velocity of the robot in configuration space}\\\hline
   \odot & \textsf{The element-wise product} \\ \hline
   \bm I & \textsf{An identity matrix} \\
\hline
\end{array}
$$

## Forward kinematic

### Relation between the positions

The problem of the forward kinematic is to find the position of the end effector given the robot configuration. This can be expressed as
$$\bm x = f(\bm q)$$

Where $f$ if an $m$ components function. In the special case of a robot arm moving in a 2D space, $f$'s components are defined as
$$f_1(\bm q) = \sum_{j = 1}^{n} l_j \cos\left( \sum_{k = 1}^{j} q_k\right)$$
$$f_2(\bm q) = \sum_{j = 1}^{n} l_j \sin\left( \sum_{k = 1}^{j} q_k\right)$$

Defining $\bm {CS}(\bm q)$ as a $2\times n$ matrix which components are the $\sin$ and $\cos$ of the robot configuration

$$
\bm {CS}(\bm q) = \left(
 \begin{matrix}
    \cos(q_1)       & 
    \cos(q_1 + q_2) & 
    \ldots          &
    \cos\left(\sum_{i = 1}^{n} q_i \right) \\ 
    \sin(q_1)       & 
    \sin(q_1 + q_2) & 
    \ldots          &
    \sin\left(\sum_{i = 1}^{n} q_i \right) \\ 
  \end{matrix}  \right)
$$

We have
$$\bm x = \bm {CS}(\bm q) \;\;\bm l$$


### Relations between the velocities
The velocities can be obtained in both spaces by differientiating both side of the forwards kinematic equation with respect to the time
$$\dot{\bm x} = \dot{f}(\bm q)$$

We can apply the chaining rule to get
$$\frac{dx_i}{dt} = \frac{df_i(\bm q)}{dt} = \sum_{j = 1}^{n} \frac{\partial f_i}{\partial q_j} \frac{dq_j}{dt} = \sum_{j = 1}^{n} \frac{\partial f_i}{\partial q_j} \dot{q_j}$$ 

Defining the Jacobian matrix
$$
\bm J(\bm q) = \left(
 \begin{matrix}
    \frac{\partial f_1}{\partial q_1} & 
    \frac{\partial f_2}{\partial q_2} & 
    \ldots\\ 
    \vdots & \vdots & \\ 
    \frac{\partial f_1}{\partial q_n} & 
    \frac{\partial f_2}{\partial q_n} & 
    \ldots\\ 
  \end{matrix}  \right)
$$

We have 
$$\dot{\bm x} = \bm J \; \dot{\bm q}$$

In the simple case defined above, the components of the Jacobian matrix are given by

$$
\begin{aligned} 
J_{1,i} &= - &\sum_{j = i}^{n}l_j\sin\left( \sum_{k = 1}^{j} q_k \right)\\
J_{2,i} &= &\sum_{j = i}^{n}l_j\cos\left( \sum_{k = 1}^{j} q_k \right)
\end{aligned}
$$

If we define $\bm L$ an $m\times m$ matrix which is the repetition of the $\bm l$ vector and $\bm\Delta$ a lower triangular $m\times m$ matrix which non zero values are equal to 1, the Jacobian can be defined in terms of the $\bm {CS}$ matrix defined in above:
$$
\bm L = \left(
 \begin{matrix}
    l_1     & l_1     & \ldots & l_1     \\
    \vdots  & \vdots  &        & \vdots  \\ 
    l_{m-1} & l_{m-1} & \ldots & l_{m-1} \\
    l_m     & l_m     & \ldots & l_m     \\
  \end{matrix}  \right) \qquad\quad
%
\bm \Delta = \left(
 \begin{matrix}
    1       &         &        &   & 0   \\
            & 1       &        &   &     \\
            &         & \ddots &         \\ 
            &         &        & 1 &     \\
    1       &         &        &   & 1   \\
  \end{matrix}  \right) \qquad\quad
%
\bm{F} = \left( \begin{matrix} 0 & -1 \\ 1 & 0  \end{matrix} \right)
\\[30pt]
\bm J(\bm q) = \bm F \;\;  \bm {CS}(\bm q) \;\; (\bm L \odot \bm \Delta)
$$

# Control

Inverting the equations above, we can control the robot to move its end-effector to a specific position. $\bm J$ being of shape $m\times n$ with, potentially, $m \neq n$, one will compute the (Moore-Penrose) pseudo inverse matrix $\bm {J^{+}} = \bm J^T (\bm{ JJ^T})^{-1}$, providing $\bm J$ is of full rank.
$$\dot{\bm q} = \bm {J^{+}} \dot{\bm x}$$

We can also add constraints to the problem, taking the advantage of the null space of $\bm J$. Indeed, let $\bm N$ being the trasformation of any vector in the configuration space to the null space of $\bm J$, the equation above becomes
$$\dot{\bm q} = \bm {J^{+}} \dot{\bm x} + \bm N  \dot{\bm \varphi}$$

Given the definition of $\bm J^{+}$, we have
$$\bm N = \bm I - \bm{J^+J}$$

Indeed
$$
\begin{aligned}
\bm {J\;N} & = \bm J (\bm I - \bm{J^+ J})   \\
           & = \bm J - \bm{J J^+ J}  \\ 
           & = \bm J - \bm J
           & =  \bm 0 
\end{aligned}
$$

Hence
$$\dot{\bm q} = \bm {J^{+}} \dot{\bm x} + (\bm I - \bm J^+ \bm J) \dot{\bm \varphi}$$

# Obstacle avoidance

$\bm{\dot{\varphi}}$ in the equation above can be computed so that the robot always move in the direction that maximizes the shortest distance between the itself and the obstacle to avoid. Let $\bm{x_0}$ be the point on the robot that is the closest to the obstacle (in the inital task space) and $\bm o$ the position of the obstacle. The desired velocity of the point $\bm{x_0}$ is given by
$$\dot{\bm {x_0}} = v_0 \frac{\bm{x_0} - \bm{o}}{\|\bm{x_0} - \bm o\|}$$

Where $v_0$ is a scalar that defines the desired velocity of the point $\bm{x_0}$ in the new one dimentional task space that is build to bring the robot away from the obstacle. Let define $\bm{d_0} = \bm{x_0} - \bm o$ and $\bm{n_0}$ a unit vector in the direction of $\bm{d_0}$. If $\bm{J_0}$ is the Jacobian relating the velocity of the point $\bm{x_0}$ to configuration of the robot, we have
$$\bm{\dot{x_0}} = \bm{J_0} \bm{\dot{q}}$$

And, therefore,
$$v_0\bm{n_0} = \bm{J_0} \bm{\dot{q}}$$

Given that $\bm{n_0}$ is of unit norm, we can find that the Jacobian that relates the robot configuration to the velocity of the point $\bm{x_0}$ is given by
$$\bm{J_{d_0}} = \bm{n_0}^T\bm{J_0}$$

In order to have a smooth transition, we want $v_0$ null when the robot is at more than a distance $d_m$ from the obstacle and increasing as it is getting closer to the obstacle:
$$v_0 = 
\begin{cases}
v_n \left( \frac{d_m^2}{\|\bm{d_0}\|^2} - 1 \right) \qquad & \textsf{if } \|\bm{d_0}\|\leq d_m \\
0 & \textsf{otherwise}
\end{cases}
$$ 

Where $v_n$ a the nominal velocity.

In the simple 2 dimentional case we started to use as en example, if $\bm{x_0}$ is on the link connecting the joint $i$ to $i + 1$ and at a distance $d$ of the joint $i$, we can define $\bm {l_0}$ and $\bm{CS_0}$ as bellow and $\bm{J_0}$ can be computed as before.
$$
\bm{l_0} =
\left(
\begin{matrix}
  l_0 \\ l_1 \\ \vdots \\ l_{i - 1} \\ d
\end{matrix}
\right) \qquad \qquad
%
\bm{CS_0}(\bm q) =
\left(
\begin{matrix}
  \cos(q_1) & \cos(q_1 + q_2) & \ldots & \cos\left(\sum_{j=1}^{i} q_j\right) \\
  \sin(q_1) & \sin(q_1 + q_2) & \ldots & \sin\left(\sum_{j=1}^{i} q_j\right) \\
\end{matrix}
\right) 
$$


## Computation of the constraint using the exact solution

If $\bm{\dot{x}}$ is the desired velocity of the end effector, recal that we have
$$\dot{\bm q} = \bm {J^{+}} \dot{\bm x} + \bm N \dot{\bm \varphi}$$

Multiplying both side by $\bm {J_{0}}$ gives
$$ \bm{\dot{x_0}} = \bm {J_0}\bm{\dot q} = \bm{J_0J^+}\bm{\dot{x}}+\bm{J_0N} \bm{\dot{\varphi}}$$
Hence
$$\bm{\dot{\varphi}} = (\bm{J_0N})^+(\bm{\dot{x_0}}-\bm{J_0J^+}\bm{\dot x})$$

Injecting this in the first equation, we get
$$
\begin{array}{lll}
  \bm{\dot{q}} & = & \bm{J^+}\bm{\dot{x}}+\bm{N\dot{\varphi}}\\
               & = & \bm{J^+}\bm{\dot{x}}+(\bm I - \bm{J^+J})\bm{\dot{\varphi}}\\
               & = & \bm{J^+}\bm{\dot{x}}+(\bm I - \bm{J^+J})(\bm{J_0N})^+(\bm{\dot{x_0}}-\bm{J_0J^+}\bm{\dot x})\\
               & = & \bm{J^+}\bm{\dot{x}} + (\bm{J_0N})^+(\bm{\dot{x_0}}-\bm{J_0J^+}\bm{\dot x}) - (\bm{J^+J})(\bm{J_0N})^+ (\bm{\dot{x_0}}-\bm{J_0J^+}\bm{\dot x}) \\
& = & \bm{J^+}\bm{\dot{x}} + (\bm{J_0N})^+(\bm{\dot{x_0}}-\bm{J_0J^+}\bm{\dot x}) \end{array}
$$

As $(\bm{J^+J})(\bm{J_0N})^+ = \bm{0}$. The last equation is valid for any task space:
 - $\bm{J_0J^+}$ projects the velocity of the end effector onto the $\cdot_0$ task space.
 - $\bm{J_0N}$ projects the configuration space onto the Jacobian null space and then onto the $\cdot_0$ space. Therefore $(\bm{J_0N})^+$ does the opposite operation, i.e. it projects elements from the $\cdot_0$ task space to the configuration space through the Jacobian null space. In other words it produces a velocity vector in the configuration space that will not influence the velocity of the end effector.

In the reduced task space defined above, we have
$$
\bm{\dot{q}} = \bm{J^+}\bm{\dot{x}} + (\bm{J_{d_0}N})^+(v_0 -\bm{J_{d_0}J^+}\bm{\dot x}) 
$$

$\bm{J_{d_0}}$ is a matrix of size $1\times n$, which makes the computations much faster.


## Approximate solution

One can also compute the velocity of the robot in configuration by combining the the velocity of the end effector and the velocity of the point $\bm{x_0}$ independently. That is $\bm{\dot{\varphi}}$ can be computed simply by inverting the Jacobian at $\bm{x_0}$:
$$\bm{\dot{\varphi}} = \bm{J_{d_0}}^+v_0$$

