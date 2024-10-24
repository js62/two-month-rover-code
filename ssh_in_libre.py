import os
import signal
import subprocess
from time import sleep
import threading


def console_send(m):
    pass


going=True

def p():
    global process, going
    while going:
        process=subprocess.Popen("ssh sbc 'bash /home/sbc2/run.sh'", shell=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        out=process.communicate()
        console_send(out[0].decode()+" "+out[1].decode())
        sleep(2)
        console_send("reconnecting to libre...")


t=threading.Thread(target=p)
t.start()




def kill():
    global going, process
    console_send("killing pid: " + str(process.pid))
    going=False
    try:
        os.killpg(os.getpgid(process.pid), signal.SIGKILL)
    except ProcessLookupError as e:
        pass
