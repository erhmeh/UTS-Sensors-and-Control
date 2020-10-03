import subprocess, shlex
command = "rosbag record"
command = shlex.split(command)
rosbag_proc = subprocess.Popen(command)