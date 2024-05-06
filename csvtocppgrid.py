# Kevin Fang - turn CSV file into a C++ vector with inclusion of packages header at the top

import csv
def csv_to_cpp_vector(csv_path, cpp_path):
    # Read the CSV file
    with open(csv_path, 'r') as file:
        reader = csv.reader(file)
        grid = [list(map(int, row)) for row in reader]

    # Write to CPP file
    with open(cpp_path, 'w') as file:
        file.write('#include <vector>\n')
        file.write('std::vector<std::vector<int>> grid = {\n')
        for row in grid:
            file.write('    {' + ', '.join(map(str, row)) + '},\n')
        file.write('};\n')

csv_to_cpp_vector('LPSR_output.csv', 'vector_grid_data.cpp') # csv_to_cpp_vector('input: CSV file path', 'output: CPP file path')
