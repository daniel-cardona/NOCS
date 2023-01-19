# Local collocation module

## Comparison of **nocs** performance with *openHRC* library and *pinocchio* library

### NAO Robot

#### *openHRC* library

| Discrete points| ODE Error | Total required time \[s\] | ipopt required time \[s\] (% of the total time) | nocs required time \[s\] (% of the total time)| 
| :---: | :---: | :---: | :---: | :---: |
| 30 | 4.83 |26.1| 13.5 (52%) | 11.6 (48%) |
| 60 | 0.17 | 79.52 | 43.1 (54%) | 34.55 (46%) |
| 90 | 0.024 | 80.9 | 43.75 (54%) | 35.53 (46%) |
| 120 | 0.012 | 81.81 | 41.32(51%) | 35.23 (49%) |

#### *pinocchio* library

| Discrete points| ODE Error | Required time \[s\] | ipopt required time \[s\] (% of the total time) | nocs required time \[s\] (% of the total time) | 
| :---: | :---: | :---: | :---: | :---: |
| 30 | 0.177 |30.8| 27.35 (89%)| 2.177 (11%) |
| 60 | 0.07 | 31.0 | 27.8 (89%)| 1.968 (11%) |
| 90 | 0.03 | 47.5 | 42.317 (89%)  | 3.402 (11%) |
| 120 | 0.017 | 91.9 | 82.470 (90%) | 6.2 (10%) |

## Mesh refinement comparison of **nocs** old version VS new version 

### NAO Robot

### *OLD version*

1. Main features of this version
   - The computation of the Jacobian of the constraints applies a dense-sparse mapping algorithm
   - Use openHRC as the library for the dynamics and gradients computation
   - Use numerical differentiation for the computation of cost function gradients
2. Numerical results (with an optimal error threshold of 1e-2)

|Mesh refinenment iter| Discrete points| ODE Error | Number of decision variables | Number of constraints | 
|:---:| :---: | :---: | :---: | :---: |
|1| 15 | 30.77 | 2090| 1441 |
|2| 29 | 3.23 | 4106| 2758 |
|3| 57 | 1.09 | 8138  | 5473 |
|4| 113 | 0.192|  16202 | 10849 |
|5| 225 | 0.0098 | 32330 | 21601 |

**TOTAL TIME REQUIRED: 5 MINUTES**

### *NEW version*

1. Main features of this version
   - Computation of the Jacobian of the constraints through **propagation algorithm**
   - Pinocchio as the library for the dynamics and gradients computation
   - Analytical differentiation for the computation of cost function gradients
2. Numerical results (with an optimal error threshold of 1e-2)

|Mesh refinenment iter| Discrete points| ODE Error | Number of decision variables | Number of constraints | 
|:---:| :---: | :---: | :---: | :---: |
|1| 15 | 4.95 | 2090| 1441 |
|2| 29 | 1.43 | 4106| 2758 |
|3| 57 | 0.10 | 8138  | 5473 |
|4| 113 | 0.0054|  16202 | 10849 |

**TOTAL TIME REQUIRED: 1.86 MINUTES**

**REDUCTION OF THE TIME IN ALMOST 63%**

