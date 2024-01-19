<!-- 底下標籤來源參考寫法可至：https://github.com/Envoy-VC/awesome-badges#github-stats -->

![](https://img.shields.io/badge/Verilog-informational)｜![](https://img.shields.io/badge/RTL-informational)｜![](https://img.shields.io/badge/Design_Compiler-informational)｜![](https://img.shields.io/badge/Synthesis-informational) ｜![](https://img.shields.io/badge/Innovus-informational)

> This is my first time to write a README.md file. I hope this file can help me to write a good README.md file in the future.

- [CVSD 2023 Fall Final Project](#CVSD-2023-Fall-Final-Project)

# CVSD 2023 Fall Final Project

> What we need to do is to implement a part of simple MIMO receiver to demodulate the RX data.    
* The architecture of MIMO receiver with 4TX*4RX is shown below.

![MIMO architecture](https://github.com/hank09901/CVSD/blob/main/Picture/MIMO.png)

> We can construct system model with AWGN(additive white Gaussian noise) channel.

### $$\underline{y} = H\underline{\breve s} + \underline{n}$$

### $$\underline{y} = \begin{bmatrix} y_1 \\ y_2 \\ y_3 \\ y_4 \end{bmatrix} = \begin{bmatrix} h_{11} & h_{12} & h_{13} & h_{14} \\ h_{21} & h_{22} & h_{23} & h_{24} \\ h_{31} & h_{32} & h_{33} & h_{34} \\ h_{41} & h_{42} & h_{43} & h_{44} \end{bmatrix} \begin{bmatrix} \breve s_1 \\ \breve s_2 \\ \breve s_3 \\ \breve s_4 \end{bmatrix} + \begin{bmatrix} n_1 \\ n_2 \\ n_3 \\ n_4 \end{bmatrix}$$

