<!-- 底下標籤來源參考寫法可至：https://github.com/Envoy-VC/awesome-badges#github-stats -->

![](https://img.shields.io/badge/Verilog-informational)｜![](https://img.shields.io/badge/RTL-informational)｜![](https://img.shields.io/badge/Design_Compiler-informational)｜![](https://img.shields.io/badge/Synthesis-informational) ｜![](https://img.shields.io/badge/Innovus-informational)

> This is my first time to write a README.md file. I hope this file can help me to write a good README.md file in the future.

- [CVSD 2023 Fall Final Project](#CVSD-2023-Fall-Final-Project)
- [01_RTL](#01_RTL)

# CVSD 2023 Fall Final Project

> What we need to do is to implement a part of simple MIMO receiver to demodulate the RX data.    
* The architecture of MIMO receiver with 4TX*4RX is shown below.

![MIMO architecture](https://github.com/hank09901/CVSD/blob/main/Picture/MIMO.png)

> We can construct system model with AWGN(additive white Gaussian noise) channel.

* $\breve s_1\,\breve s_2\,\breve s_3\,\breve s_4$ are the transmitted symbols(2 bit per symbol) with QPSK modulation. 

### $$\underline{y} = H\underline{\breve s} + \underline{n}$$

$$\underline{y} = \begin{bmatrix} y_1 \\\ y_2 \\\ y_3 \\\ y_4 \end{bmatrix} = \begin{bmatrix} h_{11} & h_{12} & h_{13} & h_{14} \\\ h_{21} & h_{22} & h_{23} & h_{24} \\\ h_{31} & h_{32} & h_{33} & h_{34} \\\ h_{41} & h_{42} & h_{43} & h_{44} \end{bmatrix} \begin{bmatrix} \breve s_1 \\\ \breve s_2 \\\ \breve s_3 \\\ \breve s_4 \end{bmatrix} + \begin{bmatrix} n_1 \\\ n_2 \\\ n_3 \\\ n_4 \end{bmatrix}$$

> MIMO receiver is composed of QR decomposition (QRD) and Maximum Likelihood (ML) demodulation

* The system model of MIMO receiver is shown below.

![MIMO system model](https://github.com/hank09901/CVSD_FinalProject/blob/main/Picture/system%20model.png)

> However, in this project, we only need to implement the QRD.

# Algorithm

1. Our algorithm is mainly based on the concept of ***modified Gram-Schmidt*** and some adjustments have been made. For each of
resource elements, we divide the algorithm into four processes. And ,we separate several computations into different operations according to what the works completed in each process.  


2. As the picture of our algorithm flow shown below, we have four procedures(iterations) such as ***PROC1***,
***PROC2***, ***PROC3***, ***PROC4***. In each process, we also have four operations, including ***SQUARE***,
***SQRT***, ***DIVIDE***, ***INNER_PRODUCT*** (not in the ***PROC4***), ***PROJECTION***.
First, in ***SQUARE*** and ***SQRT***, the norm of $h_1^\left(0\right)$, $h_2^\left(1\right)$, $h_3^\left(2\right)$, $h_4^\left(3\right)$,
namely $R_{11}$, $R_{22}$, $R_{33}$, $R_{44}$, is computed under each iteration, respectively.  


3. Next, in ***DIVIDE***, $e_1$, $e_2$, $e_3$, $e_4$ is computed for each process,
respectively. Then, in ***INNER_PRODUCT***, $R_{12}$, $R_{13}$, $R_{14}$ is computed
for ***PROC1***, $R_{23}, R_{24}, \hat{y}_1$ for ***PROC2***, and $R_{34},\hat{y}_3$ for ***PROC3***. At last,
in ***PROJECTION***, $h_2^\left(1\right)$, $h_3^\left(1\right)$, $h_4^\left(1\right)$ is computed for ***PROC1***, $h_3^\left(2\right)$, $h_4^\left(3\right)$,
$\hat{y}_2$ for PROC2, $h_4^\left(3\right)$ for ***PROC3***, and ŷ3 for PROC4.
The adjustment we make to modified Gram-Schmidt is that we
separate the computation of 4 elements of ŷ into different stages and
compute them earlier.

