# set of vehicles
set vehicle;

# set of nodes
set node1;
set node2;
set node3;
set node4;

# penalty for inaccurate arrival time
param p;

# fixed cost of each vehicle
param F;

# cost per mile unit
param cost;

# big M method
param M;

# travel time
param t{node2, node2};
param u{node3, node2};
param v{node2, node4};

# travel distance
param distance1{node2, node2};
param distance2{node3, node2};
param distance3{node2, node4};

# waiting interval
param a{node1};
param b{node1};

# Decision variables
var x{i in node2, j in node2,k in vehicle: i != j}, binary; 
var y{i in node3, j in node2,k in vehicle}, binary;
var z{i in node2, j in node4,k in vehicle}, binary;

var T{i in node1,k in vehicle}>=0;

# Objective function1
minimize time_cost:
	sum {i in node2,j in node2,k in vehicle: i != j} t[i,j] * x[i,j,k] + 
	sum {i in node3,j in node2,k in vehicle} u[i,j] * y[i,j,k] + 
	sum {i in node2,j in node4,k in vehicle} v[i,j] * z[i,j,k];

# Objective function2
minimize distance_cost:
	sum {i in node2,j in node2,k in vehicle: i != j} cost * distance1[i,j] * x[i,j,k] + 
	sum {i in node3,j in node2,k in vehicle} cost * distance2[i,j] * y[i,j,k] + 
	sum {i in node2,j in node4,k in vehicle} cost * distance3[i,j] * z[i,j,k];

# Objective function3
minimize balance_cost_time:
	sum {i in node2,j in node2,k in vehicle: i != j} cost * distance1[i,j] * x[i,j,k] + 
	sum {i in node3,j in node2,k in vehicle} cost * distance2[i,j] * y[i,j,k] + 
	sum {i in node2,j in node4,k in vehicle} cost * distance3[i,j] * z[i,j,k] -
	p * sum {i in node2, j in node2, k in vehicle: i != j} arrtime[i] * x[i,j,k] +
	p * sum {i in node2, j in node2, k in vehicle: i != j} T[i,k] * x[i,j,k] -
	p * sum {i in node2, j in node4, k in vehicle} arrtime[i] * z[i,j,k] +
	p * sum {i in node2, j in node4, k in vehicle} T[i,k] * z[i,j,k] +
	sum {i in node3,j in node2,k in vehicle} F * y[i,j,k];
	
# Constraint0: Excluding the starting point, each pick-up location needs exactly one vehicle.
subject to vehicle1{i in node2}:
	 sum{k in vehicle}sum{j in node2: j != i} x[i,j,k] + sum{k in vehicle}sum{j in node4} z[i,j,k]= 1;
	 
# Constraint1: For each vehicle, it can occupy at most one route 
# between starting point and its neighbouring points, 
subject to vehicle4{k in vehicle}:
	 sum{j in node2} y[1,j,k] <= 1;	
	 	
# Constraint2: Leave = Back
subject to vehicle2:
	sum{k in vehicle}sum{j in node2} y[1,j,k] = sum{k in vehicle}sum{i in node2} z[i,35,k];	
	
# Constraint3: Leave >=1
subject to vehicle3:
	sum{k in vehicle}sum{j in node2} y[1,j,k] >= 1;	

# Constraint4: For each vehicle at each node, Flow into = Flow out
subject to flow{k in vehicle,j in node2}:
	sum{i in node3} y[i,j,k] + sum{i in node2:i != j} x[i,j,k] = sum{i in node2: i != j} x[j,i,k] + sum{i in node4} z[j,i,k];
			
# Constraint5 waiting_time:
subject to waiting_time1 {i in node2,j in node2,k in vehicle: i != j}:
	T[i,k] + t[i,j] - T[j,k] <= M*(1 - x[i,j,k]);
	
subject to waiting_time2 {j in node2,k in vehicle}:
	T[1,k] + u[1,j] - T[j,k] <= M*(1 - y[1,j,k]);

subject to waiting_time3 {i in node2,k in vehicle}:
	T[i,k] + v[i,35] - T[35,k] <= M*(1 - z[i,35,k]);	
    
# Constraint6 waiting_interval:
subject to interval1{i in node1, k in vehicle}:
	T[i,k] >= a[i];
	
# Constraint7 waiting_interval:
subject to interval2{i in node1, k in vehicle}:
	T[i,k] <= b[i];