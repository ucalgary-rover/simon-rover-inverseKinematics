import subprocess
expectedMotorScript = "expectedMotor.py"
autoArmControllerScript = "Auto-arm-controller.py"
def run_script(script_path):
    try:
        subprocess.run(["python", script_path], check=True)
    except subprocess.CalledProcessError as e:
        print(f"Error occurred while running {script_path}")

run_script(expectedMotorScript)
run_script(autoArmControllerScript)
