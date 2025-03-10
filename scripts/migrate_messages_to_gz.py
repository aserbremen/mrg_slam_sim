import re
import sys
import os

def update_plugin_filenames(sdf_content):
    # see https://gazebosim.org/docs/harmonic/migration_from_ignition/
    # Define the find and replace patterns
    patterns_replacements = [
        (r'ign(ition)?\.gazebo', r'gz.sim'),
        (r'ign(ition)?/gazebo', r'gz/sim'),
        (r'ign(ition)?\.', r'gz.'),
        (r'ign(ition)?/', r'gz/'),
    ]

    # Perform the replacements
    updated_content = sdf_content
    for find_pattern, replace_pattern in patterns_replacements:
        updated_content = re.sub(find_pattern, replace_pattern, updated_content)

    return updated_content


def replace_libignition():

    # Update the plugin filenames
    updated_sdf_content = update_plugin_filenames(sdf_content)

    # Write the updated content back to the SDF file
    with open(sdf_path, 'w') as file:
        file.write(updated_sdf_content)

    print("Plugin filenames updated successfully.")

if __name__ == '__main__':
    if len(sys.argv) != 2:
        print("Usage: python replace_libignition.py <path_to_sdf_file>")
        sys.exit(1)

    sdf_path = sys.argv[1]
    if not os.path.isfile(sdf_path):
        print(f"Error: File '{sdf_path}' not found.")
        sys.exit(1)

    # Read the SDF file
    with open(sdf_path, 'r') as file:
        sdf_content = file.read()

    # Update the plugin filenames
    updated_sdf_content = update_plugin_filenames(sdf_content)

    # Write the updated content back to the SDF file
    with open(sdf_path, 'w') as file:
        file.write(updated_sdf_content)

    print(f"Plugin filenames updated successfully in '{sdf_path}'.")