
from time import sleep
from rich.columns import Columns
from rich.panel import Panel
from rich.live import Live
from rich.text import Text
from rich.table import Table
import socket
import pythonping
from pythonping import ping
import threading
import numpy as np



class RobotDashboard:
    def __init__(self):
        self.HOST = '192.168.1.180'
        self.PORT = 5000

        self.server = socket.socket()
        self.server.bind((self.HOST,self.PORT))
        self.server.listen()
        self.robotCounter = 0
        self.ping = []
        self.listenerThread = threading.Thread(target=self.listener,)
        self.listenerThread.start()
        self.robotNames = []
        self.robotAddresses = []

    def listener(self):
        while True:
            self.conn, self.address = self.server.accept()
            self.robotNames.append(self.getname)
            self.robotAddresses.append(self.address[0])
            self.conn.close()
            self.ping.append(ping(self.address[0])
            robotNumber = self.robotCounter
            threading.Thread(target=self.pinger, args=(robotNumber,)).start()
            self.robotCounter += 1
    def pinger(self,robotNumber):
        while True:
            self.ping[robotNumber] = ping(self.robotAddresses[robotNumber])
            sleep(0.4)
    def generate_table(self) -> Table:
        table = Table(title="")
        table.add_column("NAME", style="cyan")
        table.add_column("IP", style="magenta")
        table.add_column("PING", style="red")
        table.add_column("POSE OPTITRACK", style="green")
        table.add_column("POSE LOCAL ESTIMEATE", style="blue")
        for i in range(self.robotCounter):
            if self.ping[i].rtt_avg_ms != 2000.0:
                table.add_row(self.robotNames[i], str(self.robotAddresses[i]),str(self.ping[i].rtt_avg_ms)+" ms", "X: 0, Y:0, Z:0","X: 0, Y:0, Z:0")
            else:
                pass
        return table
    def getname(self):

        data = self.conn.recv(1024).decode()


        return(str(data))

x = RobotDashboard()

with Live(
    Panel(x.generate_table(), title="Robots", border_style="blue"),
    refresh_per_second=2,
) as live:
    while True:
        sleep(0.4)

        Panel(live.update(x.generate_table()),title = "Robots", border_style="blue")

