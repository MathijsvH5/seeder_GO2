#!/usr/bin/env python3
import subprocess
import sys
import os

def main():
    connect_script = "./connect_dog.sh"
    
    # Based on the workspace structure, the executable is likely in the build/ directory
    planting_executable = "./build/full_planting_sequence"
    
    # Fallback in case the script is run from inside the build directory
    if not os.path.exists(planting_executable) and os.path.exists("./full_planting_sequence"):
        connect_script = "../connect_dog.sh"
        planting_executable = "./full_planting_sequence"

    print(f"--- Running {connect_script} ---")
    try:
        # Run the connection script
        # Note: Since the bash script uses sudo, it might prompt for a password in the terminal
        subprocess.run([connect_script], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error executing {connect_script}. Process exited with code {e.returncode}")
        sys.exit(1)
    except FileNotFoundError:
        print(f"Error: {connect_script} not found in the current directory.")
        sys.exit(1)
        
    print("\n--- Running Planting Sequence ---")
    
    # Adding the 'eno1' argument as requested
    command = [planting_executable, "eno1"]
    print(f"Executing: {' '.join(command)}\n")
    
    try:
        # Run the C++ executable
        subprocess.run(command, check=True)
    except subprocess.CalledProcessError as e:
        print(f"\nError: {planting_executable} exited with code {e.returncode}")
        sys.exit(1)
    except FileNotFoundError:
        print(f"\nError: {planting_executable} not found. Make sure you have compiled the code using 'make'.")
        sys.exit(1)
    except KeyboardInterrupt:
        print("\nProcess interrupted by user.")
        sys.exit(0)

if __name__ == "__main__":
    main()
