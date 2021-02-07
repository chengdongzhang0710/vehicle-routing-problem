# Project Model First Edition
# VRPTW with Stages
# Model

#----------------SETS-----------------------

set N;	# nodes
set D;  # demand nodes
set K;  # vehicles
set P;  # periods
set T;  # periods except the last one

#----------------PARAMETERS-----------------

param c;  # travel cost / time unit
param F;  # fixed cost / vehicle
param J;  # penalty for late arriving / vehicle / minute
param Q;  # vehicle capacity
param s;  # service time

param d{D, P};  # demand at node i for in period p
param t{N, N};  # travel time on (i,j)
param e{P};  # lower limit of time window
param l{P};  # upper limit of time window

param M1;  # big-M coefficient, maximum of dip
param M2;  # big-M coefficient, sum of xijkp in i and j
param M3;  # big-M coefficient, 60
param M{N, N};  # big-M coefficient, maximum of (tij + s)

#----------------VARIABLES------------------

var x{i in N, j in N, k in K, p in P} binary;  # vehicle k use arc (i,j) in period p
var u{k in K, p in P} binary;  # vehicle k is used in period p

var A{i in N, k in K, p in P} >= 0;  # vehicle k's arriving time at node i in period p
var E{k in K, p in P} >= 0;  # vehicle k's ending delivery time in period p
var r{k in K} >= 0;  # vehicles k's late time after the end of planning horizon

#----------------OBJECTIVE------------------

minimize total_cost: sum{p in P, k in K} F * u[k, p] + sum{p in P, k in K, i in N, j in N} c * t[i, j] * x[i, j, k, p] + sum{k in K} J * r[k];

#----------------CONSTRAINTS----------------

subject to inner_flow_constraint{i in N, k in K, p in P}: x[i, i, k, p] = 0;

subject to total_flow_constraint1{j in D, p in P}: sum{k in K, i in N} x[i, j, k, p] <= 1;
subject to total_flow_constraint2{i in D, p in P}: sum{k in K, j in N} x[i, j, k, p] <= 1;

subject to flow_balance_equation{h in N, k in K, p in P}: sum{i in N} x[i, h, k, p] - sum{j in N} x[h, j, k, p] = 0;

subject to individual_flow_constraint1{k in K, p in P}: sum{j in D} x[0, j, k, p] <= 1;
subject to individual_flow_constraint2{k in K, p in P}: sum{i in D} x[i, 0, k, p] <= 1;

subject to vehicle_capacity_constraint1{k in K, p in P}: sum{i in D} (d[i, p] * (sum{j in N} x[i, j, k, p])) <= Q;
subject to vehicle_capacity_constraint2{i in D, p in P}: M1 * (sum{k in K, j in N} x[i, j, k, p]) >= d[i, p];

subject to link_constraint{k in K, p in P}: M2 * u[k, p] >= sum{i in N, j in N} x[i, j, k, p];

subject to time_window_constraint1{i in N, j in D, k in K, p in P}: A[i, k, p] + s + t[i, j] - A[j, k, p] <= (1 - x[i, j, k, p]) * M[i, j];
subject to time_window_constraint2{i in D, k in K, p in P}: A[i, k, p] >= e[p];
subject to time_window_constraint3{i in D, k in K, p in P}: A[i, k, p] <= l[p];
subject to time_window_constraint4{i in D, k in K, p in P}: A[i, k, p]+ s + t[i, 0] - E[k, p] <= (1 - x[i, 0, k, p]) * M[i, 0];

subject to stage_constraint1{k in K, p in T}: M3 * (1 - u[k, (p + 1)]) >= E[k, p] - 60 * (p + 1);
subject to stage_constraint2{k in K}: A[0, k, 1] = 60;
subject to stage_constraint3{k in K, p in T}: A[0, k, (p + 1)] >= E[k, p];

subject to late_arriving_constraint{k in K}: r[k] >= E[k, 4] - 300;
