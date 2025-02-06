import numpy
import pandas as pd
# import matplotlib.pyplot as plt
import seaborn as sns
# Define the number of experiments and algorithms
num_experiments = 20
# algorithms = ['RRT', 'RRTC', 'RRT*']
algorithms = ['RRTC-Joint Space','RRTC-Cartesian Space']

# Initialize an empty DataFrame with appropriate columns
# The DataFrame will have one column for each algorithm, with each row representing an experiment
df = pd.DataFrame(columns=algorithms,)

# df['RRT'] = []  # Example times
df['RRTC-Joint Space'] = [1931.48,1155.60,1444.07,933.47,1967.53,3219.85,2786.70,640.06,639.50,1194.28,
                2764.06,1428.24,1573.41,1030.33,5561.7,4796.71,2801.88,1810.29,2213.12,1241.82]  # Example times
df['RRTC-Cartesian Space'] = [1905.22,2620.36,1706.29,1425.32,1120.29,909.82,3099.77,835.78,3245.60,1821.38,
                1476.86,2555.44,1523.88,815.23,1646.46,1321.94,1247.20,4176.44,1438.81,1861.03]  # Example times

# df['RRT*'] = []
rrt=df['RRTC-Joint Space']
# rrt.plot()

# Display the empty DataFrame structure
print("DataFrame structure ready for data entry:")
print(df)

# Save to CSV (optional)
# df.to_csv('experiment_times.csv', index=True)
