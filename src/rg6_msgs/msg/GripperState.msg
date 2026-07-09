# Zustand des OnRobot RG6 am UR-Tool-Anschluss, rekonstruiert aus
# io_and_status_controller/tool_data (Analogwerte) und .../io_states (Tool-DIs).
#
# Der RG6 meldet ueber die Tool-Schnittstelle:
#   Tool-AI2 -> Greifweite (analog)
#   Tool-AI3 -> Kraft-/Motorstrom-Signal (analog)
#   Tool-DI0 -> "grip detected" (Objekt gegriffen)
#   Tool-DI1 -> "ready" (high = Ruhe, low = Bewegung laeuft) -> hier invertiert als busy

std_msgs/Header header

# Greifweite zwischen den Fingerspitzen (aus Tool-AI2, linear kalibriert
# ueber die rg6_control-Parameter width_in_open/closed + width_open/closed_m).
float64 width          # [m]
float64 width_raw      # [V] Tool-AI2 roh

# Kraftsignal (aus Tool-AI3). Rohwert; Umrechnung in N ist geraeteabhaengig.
float64 force_raw      # [V] Tool-AI3 roh

# Digitale Statussignale (nur gueltig, wenn io_states_received true ist):
bool busy              # true = Greifer bewegt sich (Tool-DI1 invertiert)
bool grip_detected     # true = Objekt gegriffen (Tool-DI0)
bool io_states_received  # io_states schon empfangen? (sonst busy/grip_detected unbelegt)
bool tool_data_received  # tool_data schon empfangen? (sonst width/force unbelegt)

# Zuletzt kommandierter Sollzustand (Treibersicht, nicht Hardware-Feedback):
bool tool_power_on     # 24-V-Toolspannung vom Treiber gesetzt
bool high_force_preset # Tool-DO1 (Kraft-Preset-Auswahl) gesetzt
uint8 last_command     # s. COMMAND_*-Konstanten

uint8 COMMAND_NONE=0
uint8 COMMAND_OPEN=1   # Tool-DO0 = 0 (Preset-Weite offen)
uint8 COMMAND_CLOSE=2  # Tool-DO0 = 1 (Preset-Weite zu)
uint8 COMMAND_GRIP=3   # Ziel-Weite/-Kraft via URScript-Injektion
