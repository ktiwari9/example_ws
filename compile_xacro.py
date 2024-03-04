import subprocess
import os
import glob

xacro_files = glob.glob('src/robot_spawn_pkg/urdf/*.urdf.xacro')

# Print the list of xacro files
print("Found xacro files:")
for xacro_file in xacro_files:
    print(f" - {xacro_file}")

for xacro_file in xacro_files:
    # Remove .xacro extension
    output_filename = os.path.splitext(xacro_file)[0]

    # Use xacro command to generate .urdf file
    subprocess.run(['xacro', xacro_file, '-o', output_filename])
