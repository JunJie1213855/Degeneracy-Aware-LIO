#!/usr/bin/env python3
"""Start every installed dataset launch and model, then check clean shutdown."""
import os
os.environ.setdefault('ROS_DOMAIN_ID', '76')
from pathlib import Path
import signal
import subprocess
import tempfile
import time


def main():
    names = ['mapping_avia_geode_alive.launch.py',
             'mapping_velodyne_geode_alive.launch.py',
             'mapping_ouster_fusion_portable_alive.launch.py',
             'mapping_velodyne_urban_nav_alive.launch.py']
    with tempfile.TemporaryDirectory(prefix='alive-startup-') as temp:
        for index,name in enumerate(names):
            logfile = Path(temp)/f'{index}.log'
            with logfile.open('w+') as output:
                process = subprocess.Popen(['ros2','launch','alive_lio',name,
                                            'rviz:=false','use_sim_time:=true'],
                                           stdout=output,stderr=subprocess.STDOUT)
                try:
                    deadline = time.monotonic()+15
                    while time.monotonic()<deadline:
                        if process.poll() is not None:
                            raise AssertionError(logfile.read_text())
                        log = logfile.read_text()
                        if 'ACC Unit' in log:
                            break
                        time.sleep(0.1)
                    else:
                        raise AssertionError('model failed to initialize: '+logfile.read_text())
                finally:
                    if process.poll() is None:
                        process.send_signal(signal.SIGINT)
                        try:
                            process.wait(timeout=10)
                        except subprocess.TimeoutExpired:
                            process.kill()
                            process.wait()
                output.seek(0)
                log = output.read()
                assert process.returncode==0 and '[ERROR]' not in log, log
                print(name+': PASS',flush=True)


if __name__=='__main__':
    main()
