
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
        self.PORT = 4008
        self.server = socket.socket()
        self.server.bind((self.HOST,self.PORT))
        self.server.listen()
        self.robotCounter = 0
        self.ping = []
        self.listenerThread = threading.Thread(target=self.listener,)
        self.listenerThread.start()
        self.robotNames = []
        self.robotAddresses = []
        self.errorCounter = []
    def listener(self):
        while True:
            self.conn, self.address = self.server.accept()
            self.robotNames.append(self.getname())
            self.robotAddresses.append(self.address[0])
            self.conn.close()
            robotNumber = self.robotCounter
            self.ping.append(ping(self.address[0]))
            threading.Thread(target=self.pinger, args=(robotNumber,)).start()
            self.errorCounter.append(0)
            self.robotCounter += 1
    def pinger(self,robotNumber):
        while True:
            self.ping[robotNumber] = ping(self.robotAddresses[robotNumber])
            sleep(0.4)
    def generate_table(self) -> Table:
        table = Table(title="")
        table.add_column("NAME")
        table.add_column("IP")
        table.add_column("PING")
        table.add_column("POSE OPTITRACK")
        table.add_column("POSE LOCAL ESTIMEATE")
        for i in range(self.robotCounter):
            if self.ping[i].rtt_avg_ms != 2000.0:
                self.errorCounter[i] = 0
                table.add_row(str("[cyan]"+self.robotNames[i])+"[/cyan]","[magenta]" +str(self.robotAddresses[i])+"[/magenta]","[red]"+str(self.ping[i].rtt_avg_ms)+ " ms" +"[/red]","[green]X: 0, Y:0, Z:0[/green]","[blue]X: 0, Y:0, Z:0[/blue]")
            elif self.errorCounter[i] < 100:
                table.add_row("[gray]" + str(self.robotNames[i]) + "[/gray]", "[gray]"+str(self.robotAddresses[i])+"[/gray]","[gray]Robot Not Found[/gray]", "[gray]X: 0, Y:0, Z:0[/gray]","[gray]X: 0, Y:0, Z:0[/gray]")
                self.errorCounter[i] +=1
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

