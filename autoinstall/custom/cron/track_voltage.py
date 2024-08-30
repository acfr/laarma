from jtop import jtop
from datetime import datetime

if __name__ == "__main__":
    with jtop() as jetson:
        if jetson.ok():
            # Get data from jtop
            power = jetson.power

            # Get formatted time
            ts = datetime.now().strftime("%y%m%d-%H%M%S")
            data = f"[{ts}] {power['rail']['VDD_CPU_CV']['volt']}"

            # Write data to voltage txt file
            with open("/home/its/autoinstall/custom/cron/voltage.txt", "a") as file:
                file.write(data + "\n")
