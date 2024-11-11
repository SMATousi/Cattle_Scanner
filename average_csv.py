import pandas as pd

c1 = pd.read_csv('generator_point_counts_1.csv')
c2 = pd.read_csv('generator_point_counts_2.csv')
c3 = pd.read_csv('generator_point_counts_3.csv')
c4 = pd.read_csv('generator_point_counts_4.csv')
c5 = pd.read_csv('generator_point_counts_5.csv')

# Extract the first column (strings) and the remaining numeric data
first_column = c1.iloc[:, 0]  # Extract the first column (strings)
header = c1.columns  # Extract the column names (header)

# Convert the remaining columns (numeric data) to float
data1 = c1.iloc[:, 1:].astype(float)
data2 = c2.iloc[:, 1:].astype(float)
data3 = c3.iloc[:, 1:].astype(float)
data4 = c4.iloc[:, 1:].astype(float)
data5 = c5.iloc[:, 1:].astype(float)

# Compute the average of the numeric values across the columns
average_data = (data1 + data2 + data3 + data4 + data5) / 5

# Combine the first column (strings) with the averaged numeric data
average_df = pd.concat([first_column.reset_index(drop=True), average_data], axis=1)

# Add the column headers back
average_df.columns = header

# Save the result to a new CSV file (optional)
average_df.to_csv('averaged_generator_point_counts.csv', index=False)

# Display the averaged data
print(average_df)
