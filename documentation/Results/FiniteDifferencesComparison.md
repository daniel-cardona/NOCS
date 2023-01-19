# Local collocation module

## Comparison of Sparse Finite Differences (SFD) Method and Block-Propagation (BP) algorithm

At this document you will find the results of the comparison between the state of the art algorithm SFD and the algorithm BP proposed by nocs. The test consist on the evaluation of the
time required for a single evaluation of the Jacobian of the constraints using finite differences methods. The SFD use an index grouping method to perturbate the non-zero elements of the sparse Jacobian only 
(for more information refer to [Curtis et al., 1974]), while the BP algorithm exploit the block structure of the Jacobian of the constraints and computes the required blocks while inserting it directly to a triplet format
using the propagation algorithm proposed at nocs.

### UR5 Robot using the pinocchio library

This test was perfomed using the version nocs-pinocchio with the UR5 Robot using the mesh refinement algorithm to solve a minimum effort problem. THe point of this test is to measure the impact of the BP Algorithm using finite differences method againts the impact of the SFD method. The result are show in the table.

Note: Just for seek of comparison a column with the required time for the using Analytical Block Propagation (ABP) algorithm is added.

#### Single evaluation fo the Jacobian of the constraints

| Discrete points| Collocation Method | Number of decision variables | Number of constraints | SFD Required time [s] | Numerical BP Required time [s] | Analytical BP Required time [s] |
| :---: | :---: | :---: | :---: | :---: | :---: |  :---: |
| 15 | H-S |524| 361| 0.004396 |0.002650|0.000396 |
| 30 | H-S |1064| 721| 0.013935 |0.005399|0.000784 |
| 60 | H-S | 2144| 1441| 0.049575|0.010986 |0.001605 |
| 90 | H-S | 3224 | 2161  | 0.104271|0.016821 |0.003092 |
| 120 | H-S | 4304 | 2881  | 0.179386|0.022517 |0.004144 |

#### Mesh refinement algorithm results


| Method| Mesh sequence | Final error | Required time [s] |
| :---: | :---: | :---: | :---: |
| SFD | 30-59-117-209 |2e-4| 215.128|
| BP | 30-59-117-209 |5.6e-4| 26.43| 
| ABP | 30-59-117-201 | 7.5e-4| 17.43| 

### NAO Robot using the pinocchio library

This test was performed using the version nocs-pinocchio with the NAO robot (24 DoF) at different number of collocation points (Average value of 10 evaluations).

Note: Just for seek of comparison a column with the required time for the computation of the analytical jacobian using BP is added.

| Discrete points| Collocation Method | Number of decision variables | Number of constraints | SFD Required time [s] | Numerical BP Required time [s] | Analytical BP Required time [s] |
| :---: | :---: | :---: | :---: | :---: | :---: |  :---: |
| 15 | H-S |2090| 1441| 0.196353 |0.040859|0.003852 |
| 30 | H-S |4252| 2881| 0.742176 |0.083778|0.010222 |
| 60 | H-S | 8570 | 5761| 2.889907|0.173725 |0.022393 |
| 90 | H-S | 12890 | 8641  | 6.196261|0.255451 |0.030520 |

