import os

# Get list of all .h files in the current directory
header_files = [file for file in os.listdir() if file.endswith('.h')]

# Sort the list of header files alphabetically
sorted_header_files = sorted(header_files)

# Print the sorted list
for header_file in sorted_header_files:
    print(header_file)