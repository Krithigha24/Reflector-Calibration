import os

# Get the current user's home directory
home_directory = os.path.expanduser("~")
# Define the relative path to the folder
relative_path = 'Reflector-Calibration/intern_ws/src/calibration/test_bags'
# Combine the home directory and the relative path
folder_path = os.path.join(home_directory, relative_path)

for filename in os.listdir(folder_path):
    if filename.startswith('bs_'):
        file_path = os.path.join(folder_path, filename)
        os.remove(file_path)
        #print(f"Deleted file: {filename}")
