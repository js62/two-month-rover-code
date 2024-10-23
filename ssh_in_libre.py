import os
import signal
import subprocess
from time import sleep
import threading

going=True

def p():
    global process, going
    while going:
        process=subprocess.Popen("ssh sbc 'bash /home/sbc2/run.sh'", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out=process.communicate()
        print(out[0].decode(),out[1].decode())
        sleep(2)
        print("reconnecting to libre...")


t=threading.Thread(target=p)
t.start()




def kill():
    global going, process
    print("killing pid:", process.pid)
    going=False
    os.killpg(os.getpgid(process.pid), signal.SIGKILL)
