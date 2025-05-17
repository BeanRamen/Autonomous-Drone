#!/usr/bin/env python3

import subprocess

def pornire_simulare():
    subprocess.Popen([
        "gnome-terminal", "--", "bash", "-c",
        "cd ~ && cd ~/PX4-Autopilot && prime-run make px4_sitl gazebo-classic_iris_rplidar__ksql_airport; exec bash"
    ])

def pornire_comunicatie():
    subprocess.Popen([
        "gnome-terminal", "--", "bash", "-c",
        "cd ~ && MicroXRCEAgent udp4 -p 8888; exec bash"
    ])

def lansare_drona():
    subprocess.Popen([
        "gnome-terminal", "--", "bash", "-c",
        "cd ~ && cd ~/ws_ros2 && source /opt/ros/humble/setup.bash && "
        "source install/local_setup.bash && source install/setup.bash && "
        "ros2 run px4_ros_com LidarDetection_Avoidance.py; exec bash"
    ])

def person_detection():
    subprocess.Popen([
        "gnome-terminal", "--", "bash", "-c",
        "cd ~ && cd ~/ws_ros2 && source /opt/ros/humble/setup.bash && "
        "source install/local_setup.bash && source install/setup.bash && "
        "ros2 run px4_ros_com YOLO_person_detector.py; exec bash"
    ])
def cartography():
    subprocess.Popen([
        "gnome-terminal", "--", "bash", "-c",
        "cd ~ && cd ~/ws_ros2 && source /opt/ros/humble/setup.bash && "
        "source install/local_setup.bash && source install/setup.bash && "
        "ros2 run px4_ros_com Cartography.py; exec bash"
    ])

def afiseaza_meniu():
    while True:
        print("\n=== MENIU ===")
        print("1. Pornire Simulare")
        print("2. Pornire Comunicație")
        print("3. Lansare Dronă")
        print("4. Person Detection")
        print("5. Cartography")
        print("0. Ieșire")
        alegere = input("Alege o opțiune: ")

        if alegere == "1":
            pornire_simulare()
        elif alegere == "2":
            pornire_comunicatie()
        elif alegere == "3":
            lansare_drona()
        elif alegere == "4":
            person_detection()
        elif alegere == "5":
            cartography()
        elif alegere == "0":
            break
        else:
            print("Opțiune invalidă.")

if __name__ == "__main__":
    afiseaza_meniu()
