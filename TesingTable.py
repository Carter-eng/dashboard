from time import sleep
from pythonping import ping
from rich.columns import Columns
from rich.panel import Panel
from rich.live import Live
from rich.text import Text
from rich.table import Table
def generate_table() -> Table:
    table = Table(title="")
    table.add_column("NAME", style="cyan")
    table.add_column("PING", style="magenta")
    table.add_column("POSE", style="green")

    table.add_row("SPOT", str(ping('10.192.239.68').rtt_avg_ms)+" ms", "X: 0, Y:0, Z:0")
    return table
with Live(
    Panel(generate_table(), title="Robots", border_style="blue"),
    refresh_per_second=2,
) as live:
    while True:
        sleep(0.4)
        
        Panel(live.update(generate_table()),title = "Robots", border_style="blue")
