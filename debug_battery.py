import subprocess
import os

def check_battery():
    print("Checking battery status...")
    
    # Try running the command exactly as we do in the server
    cmd = ["timeout", "2.0", "ros2", "topic", "echo", "/battery_state", "--field", "percentage", "--once"]
    
    print(f"Executing: {' '.join(cmd)}")
    
    # We might need to source ros2 first if it's not in the env of the subprocess
    # But for now let's try direct execution
    try:
        result = subprocess.run(
            cmd, 
            capture_output=True, 
            text=True
        )
        
        print(f"Return Code: {result.returncode}")
        print(f"Stdout: '{result.stdout.strip()}'")
        print(f"Stderr: '{result.stderr.strip()}'")
        
        if result.returncode == 0:
            print("Success!")
        else:
            print("Failed.")
            
    except Exception as e:
        print(f"Exception: {e}")

if __name__ == "__main__":
    check_battery()
