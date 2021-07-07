from time import sleep
from rich.columns import Columns
from rich.panel import Panel
from rich.live import Live
from rich.text import Text
from rich.table import Table
import socket
import pythonping
from pythonping import ping



class RobotDashboard:
    def __init__(self):
        self.HOST = '192.168.1.180'
        self.PORT = 6006

        self.server = socket.socket()
        self.server.bind((self.HOST,self.PORT))
        self.server.listen()
        self.conn, self.adresses = self.server.accept()
        self.name = self.getname()
        self.ping =ping(self.adresses[0])
    def generate_table(self) -> Table:
        table = Table(title="")
        table.add_column("NAME", style="cyan")
        table.add_column("IP", style="magenta")
        table.add_column("PING", style="red")
        table.add_column("POSE OPTITRACK", style="green")
        table.add_column("POSE LOCAL ESTIMEATE", style="blue")
        self.ping = ping(self.addresses[0])
        if self.ping.rtt_avg_ms != 2000.0:
            table.add_row(self.name, str(self.adresses[0]),str(self) "X: 0, Y:0, Z:0","X: 0, Y:0, Z:0")
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
