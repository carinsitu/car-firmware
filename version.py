import subprocess
import os
Import("env")

def get_git_version():
    try:
        # Run git describe command
        version = subprocess.check_output(
            ["git", "describe", "--always", "--dirty"], 
            stderr=subprocess.STDOUT
        ).strip().decode("utf-8")
        return version
    except:
        return "unknown"

# Get firmware version
firmware_version = get_git_version()
print(f"Firmware version: {firmware_version}")

# Define FIRMWARE_VERSION build flag
env.Append(CPPDEFINES=[
    ("FIRMWARE_VERSION", f'\\"{firmware_version}\\"')
]) 