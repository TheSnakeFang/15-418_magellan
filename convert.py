# Kevin Fang - JPG -> CSV

from PIL import Image
import numpy as np
import csv
def jpg_to_csv_file(jpg_path, csv_path):
    # Read the JPG file
    image_path = 'LPSR_85S_060M_201608.jpg'
    with Image.open(image_path) as img:
        gray = img.convert('L')

    # Convert to binary

    threshold = 128  # You can adjust this value based what you want to split to 0/1
    binary = np.array(gray) > threshold
    binary = binary.astype(int) # Convert T/F to 0/1

    # Write to CSV file
    csv_path = 'LPSR_output.csv'
    with open(csv_path, 'w', newline='') as csvfile:
        writer = csv.writer(csvfile)
        writer.writerows(binary)

jpg_to_csv_file('LPSR_85S_060M_201608.jpg', 'LPSR_output.csv') # jpg_to_csv_file('input: JPG file path', 'output: CSV file path')
