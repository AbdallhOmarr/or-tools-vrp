from ortools.constraint_solver import routing_enums_pb2
from ortools.constraint_solver import pywrapcp

def load_data():
    # lets load data
    txt_file = "vrp data.txt"
    data = []
    with open(txt_file, "r") as file:
        lines = file.readlines()  # Read all lines in the file
        column_parsed = False
        column_names = []
        if column_parsed == False:
            # Extract column names from first line
            words = re.findall("\S+", lines[0])
            for word in words:
                column_names.append(word)
            column_parsed = True

        for line in lines[1:]:  # Loop through lines, skipping the first line
            # Extract data points from line
            dataline = re.findall("\S+", line)
            line_dict = {}
            for i, point in enumerate(dataline):
                # Convert data point to float and store in dictionary
                line_dict[column_names[i]] = float(point)
            data.append(line_dict)  # Append dictionary to list of data
       


    #converting list of dict to numpy array
    data_array = np.array([[d[col] for col in column_names] for d in data])

    # #extract first row into depot array
    # DEPOT = data_array[0, :]

    # # extract rest of the rows into customers_array
    # customers_array = data_array[1:, :]
    # customers_array = add_distance_feature(customers_array,DEPOT)

    return data_array



data = load_data()

def calculate_distance_matrix(customers_array):
    n_customers = len(customers_array)
    dist_matrix = np.zeros((n_customers, n_customers))

    # Loop over each pair of customers
    for i in range(n_customers):
        for j in range(n_customers):
            # Calculate the Euclidean distance between the XCOORD and YCOORD values of the two customers
            x_diff = customers_array[i][1] - customers_array[j][1]
            y_diff = customers_array[i][2] - customers_array[j][2]
            dist = np.sqrt(np.power(x_diff, 2) + np.power(y_diff, 2))

            # Store the calculated distance in the corresponding position in the distance matrix
            dist_matrix[i][j] = dist

    return dist_matrix

def create_data_model(customers_array):
    """Stores the data for the problem."""
    demands = customers_array[:,3:4].astype(int)
    ready_time = customers_array[:,3:4].astype(int)
    data = {}
    data['READY_TIME'] = distance_matrix
    data['DUE_DATE'] = time_windows.astype(int)
    data['SERVICE_TIME'] = num_vehicles
    data['DEMAND'] = demands
    data['vehicle_capacities'] = vehicle_capacities
    data['depot'] = depot_index
    return data
    
dist_matrix = calculate_distance_matrix(customers_array)
time_windows = customers_array[:,4:6].astype(int)

num_vehicles = 20 
vehicle_capacities = [200 for x in range(num_vehicles)] 
depot_index = 0 
time_limit_seconds = 3000 # time limit for calculation

# Create routing model
manager = pywrapcp.RoutingIndexManager(len(data), 1, [1]*len(data))
routing = pywrapcp.RoutingModel(manager)

# Define distance callback
def distance_callback(from_index, to_index):
    from_node = manager.IndexToNode(from_index)
    to_node = manager.IndexToNode(to_index)
    return dist_matrix[from_node][to_node]

transit_callback_index = routing.RegisterTransitCallback(distance_callback)

# Define cost function
routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)

# Define capacity and time windows
demand = data['DEMAND'].tolist()
time_windows = [(int(data['READY_TIME'][i]), int(data['DUE_DATE'][i])) for i in range(len(data))]
service_times = data['SERVICE_TIME'].tolist()

# Add capacity constraint
capacity_callback = lambda from_index: demand[from_index]
routing.AddDimensionWithVehicleCapacity(capacity_callback, 0, [30]*len(data), True, 'Capacity')

# Add time window constraints
time_callback = lambda from_index, to_index: (
    int(time_windows[to_index][0] - dist_matrix[from_index][to_index]),
    int(time_windows[to_index][1] - dist_matrix[from_index][to_index] - service_times[to_index])
)
transit_time_callback_index = routing.RegisterTransitCallback(time_callback)
routing.AddDimensionWithVehicleCapacity(
    transit_time_callback_index,
    0,  # no slack
    [int(data['DUE_DATE'][i]) for i in range(len(data))],
    True,
    'Time'
)

# Set solution manager parameters
search_parameters = pywrapcp.DefaultRoutingSearchParameters()
search_parameters.first_solution_strategy = (
    routing_enums_pb2.FirstSolutionStrategy.AUTOMATIC
)

# Solve the problem
assignment = routing.SolveWithParameters(search_parameters)
