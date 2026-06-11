import tkinter as tk
from tkinter import ttk, scrolledtext, messagebox, filedialog
import serial
import serial.tools.list_ports
import threading
import queue
import time
from collections import defaultdict, deque
from datetime import datetime
import matplotlib
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg, NavigationToolbar2Tk
from matplotlib.figure import Figure
matplotlib.use('TkAgg')

# ===========================
# DEFINICJE PÓL TELEMETRII (kolejność i długości)
# ===========================
arraySize = 90
# name, length, scale, signed_flag, jednostka
FIELD_ORDER = [
    ("time", 22, 0.01, False, "s"),
    ("packet", 16, 1, False, "#"),
    ("errorFlags", 16, 1, False, ""),
    ("status", 3, 1, False, ""),
    ("flightStatus", 3, 1, False, ""),
    ("drogueParachute", 1, 1, False, ""),
    ("mainParachute", 1, 1, False, ""),
    ("gpsLat", 18, 1e-5, True, "°"),
    ("gpsLng", 18, 1e-5, True, "°"),
    ("gpsAlti", 18, 0.1, True, "m"),
    ("gpsHour", 5, 1, False, "h"),
    ("gpsMin", 6, 1, False, "min"),
    ("gpsSec", 6, 1, False, "s"),
    ("gpsCentisec", 7, 1, False, "s"),
    ("gpsSpeed", 15, 0.1, True, "m/s"),
    ("gpsCourse", 9, 1, False, "°"),
    ("gpsSatNum", 5, 1, False, ""),
    ("gpsHdop", 5, 1, False, ""),
    ("lsmAccelX", 15, 0.1, True, "m/s²"),
    ("lsmAccelY", 15, 0.1, True, "m/s²"),
    ("lsmAccelZ", 15, 0.1, True, "m/s²"),
    ("lsmGyroX", 15, 0.1, True, "°/s"),
    ("lsmGyroY", 15, 0.1, True, "°/s"),
    ("lsmGyroZ", 15, 0.1, True, "°/s"),
    ("lsmTemp", 10, 1, True, "°C"),
    ("lsmAlti", 18, 0.1, True, "m"),
    ("lsmSpeed", 15, 0.1, True, "m/s"),
    ("lsmAccel", 15, 0.1, True, "m/s²"),
    ("adxlAccelX", 16, 0.1, True, "m/s²"),
    ("adxlAccelY", 16, 0.1, True, "m/s²"),
    ("adxlAccelZ", 16, 0.1, True, "m/s²"),
    ("adxlAlti", 18, 0.1, True, "m"),
    ("adxlSpeed", 15, 0.1, True, "m/s"),
    ("adxlAccel", 16, 0.1, True, "m/s²"),
    ("bmp1Temp", 10, 1, True, "°C"),
    ("bmp1Press", 11, 1, False, "hPa"),
    ("bmp1Alti", 18, 0.1, True, "m"),
    ("bmp1Speed", 15, 0.1, True, "m/s"),
    ("bmp2Temp", 10, 1, True, "°C"),
    ("bmp2Press", 11, 1, False, "hPa"),
    ("bmp2Alti", 18, 0.1, True, "m"),
    ("bmp2Speed", 15, 0.1, True, "m/s"),
    ("battery", 6, 0.1, False, "V"),
    ("filteredAccelX", 16, 0.1, True, "m/s²"),
    ("filteredAccelY", 16, 0.1, True, "m/s²"),
    ("filteredAccelZ", 16, 0.1, True, "m/s²"),
    ("filteredGyroX", 15, 0.1, True, "°/s"),
    ("filteredGyroY", 15, 0.1, True, "°/s"),
    ("filteredGyroZ", 15, 0.1, True, "°/s"),
    ("filteredAlti", 18, 0.1, True, "m"),
    ("filteredSpeed", 15, 0.1, True, "m/s"),
    ("filteredAccel", 16, 0.1, True, "m/s²"),
    ("filteredRoll", 8, 1, True, "°"),
    ("filteredPitch", 8, 1, True, "°"),
]


# ===========================
# PARSER BINARNY
# ===========================
class TelemetryParser:
    @staticmethod
    def parse_packet(data_bytes):
        if len(data_bytes) != arraySize:
            return None
        if data_bytes[0] != 0xFF or data_bytes[1] != 0x66:
            return None
        crc = 0
        for i in range(arraySize-1):
            crc ^= data_bytes[i]
        if crc != data_bytes[arraySize-1]:
            return None

        data_part = data_bytes[2:arraySize-1]
        # Kolejność bitów: MSB pierwszego bajta jako pierwszy (zgodnie z setBit)
        bits = ''.join(f'{b:08b}' for b in data_part)

        result = {}
        pos = 0
        for name, length, scale, signed_flag, _ in FIELD_ORDER:
            if pos + length > len(bits):
                break

            if signed_flag:
                # Format: 1 bit znaku + (length-1) bitów wartości bezwzględnej
                sign_bit = int(bits[pos], 2)      # 0 = dodatnia, 1 = ujemna
                abs_bits = bits[pos+1:pos+length]
                abs_val = int(abs_bits, 2)
                raw = -abs_val if sign_bit else abs_val
            else:
                raw = int(bits[pos:pos+length], 2)

            result[name] = raw * scale
            pos += length

        return result


# ===========================
# WĄTEK ODCZYTU Z SERIALU
# ===========================
class SerialReaderThread(threading.Thread):
    def __init__(self, port, baudrate, data_queue, stop_event):
        super().__init__(daemon=True)
        self.port = port
        self.baudrate = baudrate
        self.data_queue = data_queue
        self.stop_event = stop_event
        self.serial = None

    def run(self):
        try:
            self.serial = serial.Serial(self.port, self.baudrate, timeout=0.1)
            buffer = bytearray()
            while not self.stop_event.is_set():
                if self.serial.in_waiting:
                    chunk = self.serial.read(self.serial.in_waiting)
                    buffer.extend(chunk)
                    while len(buffer) >= arraySize:
                        idx = -1
                        for i in range(len(buffer)-1):
                            if buffer[i] == 0xFF and buffer[i+1] == 0x66:
                                idx = i
                                break
                        if idx == -1:
                            text_data = buffer.decode(errors='ignore').split('\n')
                            for line in text_data[:-1]:
                                if line.strip():
                                    self.data_queue.put(('text', line.strip()))
                            buffer.clear()
                            break
                        if idx > 0:
                            prefix = buffer[:idx]
                            text_data = prefix.decode(errors='ignore').split('\n')
                            for line in text_data[:-1]:
                                if line.strip():
                                    self.data_queue.put(('text', line.strip()))
                            buffer = buffer[idx:]
                            continue
                        if len(buffer) >= arraySize:
                            packet = buffer[:arraySize]
                            buffer = buffer[arraySize:]
                            self.data_queue.put(('binary', packet))
                        else:
                            break
                else:
                    time.sleep(0.01)
        except Exception as e:
            self.data_queue.put(('error', str(e)))
        finally:
            if self.serial and self.serial.is_open:
                self.serial.close()


# ===========================
# GŁÓWNA APLIKACJA
# ===========================
class RocketGroundStation:
    def __init__(self, root):
        self.root = root
        self.root.title("Rocket Telemetry Ground Station")
        self.root.geometry("1400x700")

        # Zmienne
        self.serial_thread = None
        self.stop_event = threading.Event()
        self.data_queue = queue.Queue()
        self.running = True
        self.telemetry_data = {}
        self.history = defaultdict(lambda: deque(maxlen=500))
        self.packet_counter = 0
        self.last_rocket_packet = -1
        self.lost_packets = 0
        self.recording = False
        self.csv_file = None
        self.csv_buffer = []
        self.csv_buffer_size = 0
        self.simulation_mode = False
        self.sim_timer = None
        self.sim_packet_counter = 0

        # Kontrola odświeżania wykresów
        self.last_plot_update = 0
        self.plot_update_interval = 50  # ms
        self.points_limit = 1500

        # Główny kontener z przewijaniem
        self.main_canvas = tk.Canvas(self.root, highlightthickness=0)
        self.scrollbar = ttk.Scrollbar(self.root, orient=tk.VERTICAL, command=self.main_canvas.yview)
        self.scrollable_frame = ttk.Frame(self.main_canvas)
        self.scrollable_frame.bind("<Configure>",
                                   lambda e: self.main_canvas.configure(scrollregion=self.main_canvas.bbox("all")))
        self.canvas_window_id = self.main_canvas.create_window((0, 0), window=self.scrollable_frame, anchor=tk.NW)
        self.main_canvas.configure(yscrollcommand=self.scrollbar.set)
        self.main_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        self.scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        def _on_mousewheel(event):
            self.main_canvas.yview_scroll(int(-1 * (event.delta / 120)), "units")

        self.main_canvas.bind_all("<MouseWheel>", _on_mousewheel)
        self.main_canvas.bind("<Configure>", self._configure_canvas)

        self.setup_gui()
        self.update_from_queue()

    def _configure_canvas(self, event):
        self.main_canvas.itemconfig(self.canvas_window_id, width=event.width)

    def setup_gui(self):
        # Górny pasek statusu
        status_frame = ttk.LabelFrame(self.scrollable_frame, text="Status", padding=5)
        status_frame.pack(fill=tk.X, padx=5, pady=5)
        self.status_label = ttk.Label(status_frame, text="Status: - | Mode: Not Connected", font=('Arial', 10, 'bold'))
        self.status_label.pack(side=tk.LEFT)
        self.connection_label = ttk.Label(status_frame, text="Disconnected", foreground="red", font=('Arial', 10, 'bold'))
        self.connection_label.pack(side=tk.RIGHT)

        # Główny układ poziomy
        main_horizontal = ttk.Frame(self.scrollable_frame)
        main_horizontal.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        left_frame = ttk.Frame(main_horizontal)
        left_frame.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        right_frame = ttk.Frame(main_horizontal, width=480)
        right_frame.pack(side=tk.RIGHT, fill=tk.BOTH, expand=False)
        right_frame.pack_propagate(False)

        # ---------- LEWA CZĘŚĆ: WYKRESY ----------
        plots_frame = ttk.LabelFrame(left_frame, text="Wykresy telemetrii", padding=5)
        plots_frame.pack(fill=tk.BOTH, expand=True)

        self.fig = Figure(figsize=(10, 5), dpi=100)
        self.fig.subplots_adjust(hspace=0.4, top=0.95, bottom=0.12, left=0.08, right=0.92)

        self.ax_acc = self.fig.add_subplot(2, 2, 1)
        self.ax_vel = self.fig.add_subplot(2, 2, 2)
        self.ax_alt = self.fig.add_subplot(2, 2, 3)
        self.ax_press_temp = self.fig.add_subplot(2, 2, 4)

        # Konfiguracja osi
        for ax, title in zip([self.ax_acc, self.ax_vel, self.ax_alt],
                             ["Przyspieszenie", "Prędkość", "Wysokość"]):
            ax.set_xlabel("Paczka")
            ax.grid(True, alpha=0.3)
            ax.set_title(title)
        self.ax_acc.set_ylabel("m/s²")
        self.ax_vel.set_ylabel("m/s")
        self.ax_alt.set_ylabel("m")

        # Wykres ciśnienia + temperatury (druga oś Y)
        self.ax_press_temp.set_title("Ciśnienie i Temperatura")
        self.ax_press_temp.set_xlabel("Paczka")
        self.ax_press_temp.set_ylabel("hPa", color='blue')
        self.ax_press_temp.tick_params(axis='y', labelcolor='blue')
        self.ax_press_temp.grid(True, alpha=0.3)

        self.ax_temp = self.ax_press_temp.twinx()
        self.ax_temp.set_ylabel("°C", color='red')
        self.ax_temp.tick_params(axis='y', labelcolor='red')

        self.ax_acc.set_title("Przyspieszenie")
        self.ax_acc.set_ylabel("m/s²")
        self.ax_vel.set_title("Prędkość")
        self.ax_vel.set_ylabel("m/s")
        self.ax_alt.set_title("Wysokość")
        self.ax_alt.set_ylabel("m")
        for ax in [self.ax_acc, self.ax_vel, self.ax_alt, self.ax_press_temp]:
            ax.set_xlabel("Paczka")
            ax.grid(True, alpha=0.3)
        self.ax_press_temp.set_title("Ciśnienie i Temperatury")
        self.ax_press_temp.set_ylabel("hPa", color='blue')
        self.ax_press_temp.tick_params(axis='y', labelcolor='blue')
        self.ax_temp.set_ylabel("°C", color='red')
        self.ax_temp.tick_params(axis='y', labelcolor='red')

        self.canvas = FigureCanvasTkAgg(self.fig, master=plots_frame)
        self.canvas.draw()

        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        toolbar = NavigationToolbar2Tk(self.canvas, plots_frame)
        toolbar.update()

        # ---------- PRAWA CZĘŚĆ: PANEL DANYCH (3 kolumny, przewijany) ----------
        data_panel = ttk.LabelFrame(right_frame, text="Wszystkie dane telemetryczne", padding=5)
        data_panel.pack(fill=tk.BOTH, expand=True)

        # Kontener z przewijaniem dla danych
        data_canvas = tk.Canvas(data_panel, highlightthickness=0)
        data_scrollbar = ttk.Scrollbar(data_panel, orient=tk.VERTICAL, command=data_canvas.yview)
        data_scrollable = ttk.Frame(data_canvas)
        data_scrollable.bind("<Configure>", lambda e: data_canvas.configure(scrollregion=data_canvas.bbox("all")))
        data_canvas.create_window((0, 0), window=data_scrollable, anchor=tk.NW)
        data_canvas.configure(yscrollcommand=data_scrollbar.set)
        data_canvas.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        data_scrollbar.pack(side=tk.RIGHT, fill=tk.Y)

        # Etykiety dla wszystkich pól (3 kolumny, większy font)
        exclude_fields = {
            # błędy
            'errorFlags',
            'lsmAccelX', 'lsmAccelY', 'lsmAccelZ', 'lsmAccel', 'adxlAccelX', 'adxlAccelY', 'adxlAccelZ', 'adxlAccel',
            'filteredAccelX', 'filteredAccelY', 'filteredAccelZ', 'filteredAccel',
            'lsmSpeed', 'adxlSpeed', 'bmp1Speed', 'bmp2Speed', 'gpsSpeed', 'filteredSpeed',
            'lsmAlti', 'adxlAlti', 'bmp1Alti', 'bmp2Alti', 'gpsAlti', 'filteredAlti',
            'bmp1Press', 'bmp2Press',
            'lsmTemp', 'bmp1Temp', 'bmp2Temp'
        }

        filtered_fields = [(name, unit) for name, _, _, _, unit in FIELD_ORDER if name not in exclude_fields]
        self.data_labels = {}
        row, col = 0, 0

        extended_fields = []
        for name, unit in filtered_fields:
            extended_fields.append((name, unit))
            if name in ['battery', 'gpsHdop', 'lsmGyroZ', 'filteredGyroZ']:
                extended_fields.append((None, None))

        for name, unit in extended_fields:
            if name is not None:
                frame = ttk.Frame(data_scrollable, relief=tk.RIDGE, borderwidth=1)
                frame.grid(row=row, column=col, padx=2, pady=2, sticky="nsew")
                ttk.Label(frame, text=f"{name}:", font=('Arial', 10, 'bold')).pack(side=tk.LEFT, padx=2)
                label_val = ttk.Label(frame, text="---", width=10, anchor=tk.E, font=('Arial', 10, 'bold'))
                label_val.pack(side=tk.RIGHT, padx=2)
                ttk.Label(frame, text=f"{unit}", font=('Arial', 9, 'bold')).pack(side=tk.RIGHT)
                self.data_labels[name] = label_val
            col += 1
            if col >= 2:
                col = 0
                row += 1

        # Informacja o pakietach
        packet_frame = ttk.Frame(data_scrollable, relief=tk.RIDGE)
        packet_frame.grid(row=row + 1, column=0, columnspan=2, sticky="nsew", pady=5)
        self.packet_info_label = ttk.Label(packet_frame, text="Pakiety: 0 odebrane, 0 strat", foreground="blue")
        self.packet_info_label.pack(pady=2)

        # Lista błędów (kolorowanie)
        error_frame = ttk.LabelFrame(data_scrollable, text="Aktywne błędy", padding=5)
        error_frame.grid(row=row + 2, column=0, columnspan=2, sticky="nsew", pady=5)
        self.error_labels = {}
        errors_list = [
            "LORA_ERROR", "LSM_ERROR", "BMP1_ERROR", "BMP2_ERROR",
            "ADXL_ERROR", "GPS_ERROR", "FLASH_ERROR", "FLASH_FILE_ERROR", "MSG_TOO_LONG_ERROR"
        ]
        for i, err_name in enumerate(errors_list):
            lbl = ttk.Label(error_frame, text=f"{err_name}: OK", foreground="green")
            lbl.grid(row=i // 3, column=i % 3, padx=5, pady=2, sticky="w")
            self.error_labels[err_name] = lbl

        # ---------- SEKCJA KOMEND ----------
        cmd_frame = ttk.LabelFrame(self.scrollable_frame, text="Wysyłanie komend", padding=10)
        cmd_frame.pack(fill=tk.X, padx=5, pady=5)

        # Dowolna komenda
        row1 = ttk.Frame(cmd_frame)
        row1.pack(fill=tk.X, pady=2)
        ttk.Label(row1, text="Dowolna komenda:").pack(side=tk.LEFT, padx=5)
        self.custom_cmd_var = tk.StringVar()
        self.custom_cmd_entry = ttk.Entry(row1, textvariable=self.custom_cmd_var, width=40)
        self.custom_cmd_entry.pack(side=tk.LEFT, fill=tk.X, expand=True, padx=5)
        ttk.Button(row1, text="Wyślij", command=self.send_custom_command).pack(side=tk.LEFT, padx=5)

        # Szybkie komendy (przyciski)
        quick_commands = [
            "RESET", "RESET_OFFSETS", "SET_OFFSETS", "GET_RAW_DATA",
            "INIT_WATCHDOG", "DEPLOY_DROGUE", "DEPLOY_MAIN", "VERSION",
            "GET_GPS_OFFSET", "GET_OFFSETS", "GET_LORA_STATUS", "GET_DATA",
            "HELP", "GET_STATUS", "GET_ERRORS", "CLEAR_ERRORS"
        ]
        quick_frame = ttk.Frame(cmd_frame)
        quick_frame.pack(fill=tk.X, pady=5)
        for i, cmd in enumerate(quick_commands):
            btn = ttk.Button(quick_frame, text=cmd, command=lambda c=cmd: self.send_quick_command(c))
            btn.grid(row=i // 8, column=i % 8, padx=2, pady=2, sticky="ew")

        # Komendy z parametrami
        param_frame = ttk.Frame(cmd_frame)
        param_frame.pack(fill=tk.X, pady=5)
        ttk.Label(param_frame, text="SET_MODE:").pack(side=tk.LEFT, padx=5)
        self.mode_var = tk.StringVar()
        mode_combo = ttk.Combobox(param_frame, textvariable=self.mode_var, values=["DEBUG", "FLIGHT", "DUMP", "SLEEP"], width=8)
        mode_combo.pack(side=tk.LEFT, padx=2)
        ttk.Button(param_frame, text="Wyślij", command=lambda: self.send_quick_command(f"SET_MODE {self.mode_var.get()}")).pack(side=tk.LEFT, padx=5)
        ttk.Label(param_frame, text="SET_FLIGHT_STATE:").pack(side=tk.LEFT, padx=5)
        self.state_var = tk.StringVar()
        state_combo = ttk.Combobox(param_frame, textvariable=self.state_var, values=["IDLE", "BOOST", "COAST", "APOGEE", "DESCENT", "LANDED"], width=10)
        state_combo.pack(side=tk.LEFT, padx=2)
        ttk.Button(param_frame, text="Wyślij", command=lambda: self.send_quick_command(f"SET_FLIGHT_STATE {self.state_var.get()}")).pack( side=tk.LEFT, padx=5)

        # ---------- LOG ----------
        log_frame = ttk.LabelFrame(self.scrollable_frame, text="Log zdarzeń", padding=5)
        log_frame.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self.log_text = scrolledtext.ScrolledText(log_frame, height=10, font=('Courier', 9), wrap=tk.WORD)
        self.log_text.pack(fill=tk.BOTH, expand=True)
        self.log_text.tag_config('error', foreground='red')
        self.log_text.tag_config('warning', foreground='orange')
        self.log_text.tag_config('success', foreground='green')
        self.log_text.tag_config('sent', foreground='blue')
        self.log_text.tag_config('received', foreground='purple')

        # ---------- DOLNY PASEK KONTROLEK ----------
        control_frame = ttk.Frame(self.scrollable_frame)
        control_frame.pack(fill=tk.X, padx=5, pady=5)

        ttk.Label(control_frame, text="Port:").pack(side=tk.LEFT, padx=5)
        self.port_var = tk.StringVar()
        ports = [port.device for port in serial.tools.list_ports.comports()]
        self.port_combo = ttk.Combobox(control_frame, textvariable=self.port_var, values=ports, width=10)
        self.port_combo.pack(side=tk.LEFT, padx=5)
        if ports:
            self.port_combo.set(ports[0])

        ttk.Label(control_frame, text="Baud:").pack(side=tk.LEFT, padx=5)
        self.baud_var = tk.StringVar(value="115200")
        baud_combo = ttk.Combobox(control_frame, textvariable=self.baud_var, values=["9600", "19200", "38400", "57600", "115200"], width=8)
        baud_combo.pack(side=tk.LEFT, padx=5)

        self.connect_btn = ttk.Button(control_frame, text="Połącz", command=self.toggle_connection)
        self.connect_btn.pack(side=tk.LEFT, padx=10)

        ttk.Button(control_frame, text="Wyczyść wykresy", command=self.clear_plots).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Wyczyść wszystko", command=self.clear_all_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Zapisz CSV", command=self.save_to_csv).pack(side=tk.LEFT, padx=5)
        self.record_btn = ttk.Button(control_frame, text="Rozpocznij zapis", command=self.toggle_recording)
        self.record_btn.pack(side=tk.LEFT, padx=5)
        self.sim_var = tk.BooleanVar(value=False)
        sim_check = ttk.Checkbutton(control_frame, text="Symulacja", variable=self.sim_var, command=self.toggle_simulation)
        sim_check.pack(side=tk.LEFT, padx=10)

        self.plot_lines = {}
        self.init_plot_lines()

    def init_plot_lines(self):
        # Konfiguracja osi (tylko raz)
        self.ax_acc.set_title("Przyspieszenie (LSM, ADXL, filt)")
        self.ax_acc.set_ylabel("m/s²")
        self.ax_vel.set_title("Prędkość (wszystkie sensory)")
        self.ax_vel.set_ylabel("m/s")
        self.ax_alt.set_title("Wysokość (wszystkie sensory)")
        self.ax_alt.set_ylabel("m")
        for ax in [self.ax_acc, self.ax_vel, self.ax_alt, self.ax_press_temp]:
            ax.set_xlabel("Paczka")
            ax.grid(True, alpha=0.3)
        self.ax_press_temp.set_title("Ciśnienie i Temperatury")
        self.ax_press_temp.set_ylabel("hPa", color='blue')
        self.ax_press_temp.tick_params(axis='y', labelcolor='blue')
        self.ax_temp.set_ylabel("°C", color='red')
        self.ax_temp.tick_params(axis='y', labelcolor='red')

        # Tworzenie linii dla każdej serii danych
        self.plot_lines = {}
        # Przyspieszenia
        for prefix in ['lsm', 'adxl']:
            for axis in ['X', 'Y', 'Z']:
                key = f"{prefix}Accel{axis}"
                line, = self.ax_acc.plot([], [], 'o', label=f"{prefix.upper()}-{axis}", markersize=1, alpha=0.7)
                self.plot_lines[key] = line
        for axis in ['X', 'Y', 'Z']:
            key = f"filteredAccel{axis}"
            line, = self.ax_acc.plot([], [], 'o', label=f"filt-{axis}", markersize=1, color='purple')
            self.plot_lines[key] = line
        # Prędkości
        for key in ['lsmSpeed', 'adxlSpeed', 'bmp1Speed', 'bmp2Speed', 'gpsSpeed', 'filteredSpeed']:
            line, = self.ax_vel.plot([], [], 'o', label=key.replace('Speed', ''), markersize=1, alpha=0.7)
            self.plot_lines[key] = line
        # Wysokości
        for key in ['lsmAlti', 'adxlAlti', 'bmp1Alti', 'bmp2Alti', 'gpsAlti', 'filteredAlti']:
            line, = self.ax_alt.plot([], [], 'o', label=key.replace('Alti', ''), markersize=1, alpha=0.7)
            self.plot_lines[key] = line
        # Ciśnienia i temperatury
        for key in ['bmp1Press', 'bmp2Press']:
            line, = self.ax_press_temp.plot([], [], 'o', label=key, color='blue', markersize=2)
            self.plot_lines[key] = line
        for key in ['lsmTemp', 'bmp1Temp', 'bmp2Temp']:
            line, = self.ax_temp.plot([], [], 'o', label=key, color='red', markersize=2)
            self.plot_lines[key] = line

        # Legendy (tylko raz)
        self.ax_acc.legend(loc='upper left', fontsize=7)
        self.ax_vel.legend(loc='upper left', fontsize=7)
        self.ax_alt.legend(loc='upper left', fontsize=7)
        lines1, labels1 = self.ax_press_temp.get_legend_handles_labels()
        lines2, labels2 = self.ax_temp.get_legend_handles_labels()
        self.ax_press_temp.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=7)

    def toggle_simulation(self):
        if self.sim_var.get():
            if self.serial_thread and self.serial_thread.is_alive():
                self.toggle_connection()
            self.simulation_mode = True
            self.log("Tryb symulacji WŁĄCZONY", 'warning')
            self.start_simulation()
        else:
            self.simulation_mode = False
            self.stop_simulation()
            self.log("Tryb symulacji WYŁĄCZONY", 'warning')

    def start_simulation(self):
        if not self.simulation_mode:
            return
        self.generate_sim_packet()
        self.sim_timer = self.root.after(200, self.start_simulation)

    def stop_simulation(self):
        if self.sim_timer:
            self.root.after_cancel(self.sim_timer)
            self.sim_timer = None

    def generate_sim_packet(self):
        self.sim_packet_counter += 1
        if self.sim_packet_counter % 68 == 0:
            return
        t = self.sim_packet_counter * 0.1
        e = 0
        if self.sim_packet_counter % 10 == 0:
            e |= (1 << 0)
        if self.sim_packet_counter % 5 == 0:
            e |= (1 << 3)
        data = {
            'packet': self.sim_packet_counter,
            'time': t,
            'status': 1 + self.sim_packet_counter % 4,
            'flightStatus': 1 + self.sim_packet_counter % 6,
            'drogueParachute': 1 if t > 3 else 0,
            'mainParachute': 1 if t > 5 else 0,
            'errorFlags': e,
            'gpsLat': 0.0 + 0.0001 * self.sim_packet_counter,
            'gpsLng': 0.0 + 0.0001 * self.sim_packet_counter,
            'gpsAlti': 100 + t * 10,
            'gpsHour': 12, 'gpsMin': 0, 'gpsSec': int(t) % 60, 'gpsCentisec': int((t % 1) / 10),
            'gpsSpeed': t * 2,
            'gpsCourse': 90,
            'gpsSatNum': 10,
            'gpsHdop': 2,
            'lsmAccelX': 0.05 * t, 'lsmAccelY': 0.5 * t, 'lsmAccelZ': -0.02 * t,
            'lsmGyroX': 0, 'lsmGyroY': 0, 'lsmGyroZ': 0,
            'lsmTemp': 25 + 0.1 * t,
            'lsmAlti': 100 + t * 10.02,
            'lsmSpeed': t * 20.1,
            'lsmAccel': 0.55 * t,
            'adxlAccelX': 0.05 * t, 'adxlAccelY': 0.48 * t, 'adxlAccelZ': -0.02 * t,
            'adxlAlti': 103 + t * 10.05,
            'adxlSpeed': t * 20.1,
            'adxlAccel': 0.52 * t,
            'bmp1Temp': 24 + 0.1 * t,
            'bmp1Press': 1013 - 0.02 * t,
            'bmp1Alti': 105 + t * 9.80,
            'bmp1Speed': t * 19.9,
            'bmp2Temp': 24 + 0.1 * t,
            'bmp2Press': 1013 - 0.02 * t,
            'bmp2Alti': 105 + t * 8.85,
            'bmp2Speed': t * 19.95,
            'battery': 4.2 - 0.001 * t,
            'filteredAccelX': 0.05 * t, 'filteredAccelY': 0.5 * t, 'filteredAccelZ': -0.02 * t,
            'filteredGyroX': 0, 'filteredGyroY': 0, 'filteredGyroZ': 0,
            'filteredAlti': 100 + t * 10,
            'filteredSpeed': t * 20,
            'filteredAccel': 0.5 * t,
            'filteredRoll': 5 * t,
            'filteredPitch': 2 * t,
        }
        packet_bytes = self.encode_telemetry_packet(data)
        if packet_bytes:
            self.process_binary_packet(packet_bytes)

    def encode_telemetry_packet(self, data):
        """Konwertuje słownik wartości fizycznych na arraySize-bajtowy pakiet (header 0xFF66 + dane + CRC)"""
        # Kolejność pól i ich długości (takie same jak FIELD_ORDER)
        bits_list = []
        pos = 0
        for name, length, scale, signed_flag, _ in FIELD_ORDER:
            if name not in data:
                return None
            raw_value = int(data[name] / scale)
            if signed_flag and raw_value < 0:
                raw_value = (1 << length) + raw_value  # konwersja na uzupełnienie do 2
            # Sprawdź zakres
            max_val = (1 << length) - 1
            min_val = 0 if not signed_flag else -(1 << (length - 1))
            if raw_value < 0 or raw_value > max_val:
                print(f"Warning: {name} value {raw_value} out of range for {length} bits")
                raw_value = max(0, min(max_val, raw_value))
            # Dodaj bity do listy
            bits_list.append(f"{raw_value:0{length}b}")
            pos += length

        full_bits = ''.join(bits_list)
        # Dopełnij do pełnej liczby bajtów (86 bajtów = 688 bitów)
        expected_bits = sum(length for _, length, _, _, _ in FIELD_ORDER)
        if len(full_bits) != expected_bits:
            print(f"Bit length mismatch: {len(full_bits)} vs {expected_bits}")
            return None

        # Konwersja bitów na bajty
        data_bytes = bytearray()
        for i in range(0, len(full_bits), 8):
            byte_bits = full_bits[i:i + 8]
            if len(byte_bits) < 8:
                byte_bits = byte_bits.ljust(8, '0')
            data_bytes.append(int(byte_bits, 2))

        if len(data_bytes) != 86:
            print(f"Data bytes length mismatch: {len(data_bytes)} vs 86")
            return None

        # Dodaj header 0xFF66
        packet = bytearray([0xFF, 0x66]) + data_bytes
        # Oblicz CRC (XOR wszystkich bajtów oprócz ostatniego)
        crc = 0
        for b in packet:
            crc ^= b
        packet.append(crc)  # dodaj CRC na koniec

        if len(packet) != arraySize:
            print(f"Packet length mismatch: {len(packet)} vs {arraySize}")
            return None

        return bytes(packet)

    # ------------------ METODY POMOCNICZE (log, send_serial, itd.) ------------------
    def log(self, message, tag=None):
        timestamp = datetime.now().strftime("%H:%M:%S")
        line = f"[{timestamp}] {message}\n"
        self.log_text.insert(tk.END, line)
        if tag:
            end = self.log_text.index("end-1c")
            start = f"{end} - {len(message) + len(timestamp) + 3} chars"
            self.log_text.tag_add(tag, start, end)
        self.log_text.see(tk.END)

    def send_serial(self, command):
        if self.serial_thread and self.serial_thread.serial and self.serial_thread.serial.is_open:
            try:
                cmd = command.strip() + '\n'
                self.serial_thread.serial.write(cmd.encode())
                self.log(f"WYSŁANO: {command}", 'sent')
                return True
            except Exception as e:
                self.log(f"Błąd wysyłania: {e}", 'error')
        else:
            self.log("Brak połączenia", 'error')
        return False

    def send_custom_command(self):
        cmd = self.custom_cmd_var.get().strip()
        if cmd:
            self.send_serial(cmd)
            self.custom_cmd_var.set("")
        else:
            self.log("Pusta komenda", 'warning')

    def send_quick_command(self, cmd):
        self.send_serial(cmd)

    def toggle_connection(self):
        if self.serial_thread and self.serial_thread.is_alive():
            self.stop_event.set()
            self.serial_thread.join(timeout=1)
            self.serial_thread = None
            self.connect_btn.config(text="Połącz")
            self.connection_label.config(text="Disconnected", foreground="red")
            self.status_label.config(text="Status: - | Mode: Not Connected")
            self.log("Rozłączono", 'warning')
        else:
            port = self.port_var.get().strip()
            baud = int(self.baud_var.get())
            if not port:
                messagebox.showerror("Błąd", "Wybierz port")
                return
            self.stop_event.clear()
            self.serial_thread = SerialReaderThread(port, baud, self.data_queue, self.stop_event)
            self.serial_thread.start()
            self.connect_btn.config(text="Rozłącz")
            self.connection_label.config(text=f"Connected: {port}", foreground="green")
            self.status_label.config(text=f"Status: Connected | Mode: Serial")
            self.log(f"Połączono z {port} (baud {baud})", 'success')
            self.packet_counter = 0
            self.last_rocket_packet = -1
            self.lost_packets = 0
            self.update_packet_info()

    def update_from_queue(self):
        if not self.running:
            return
        try:
            while True:
                item = self.data_queue.get_nowait()
                if item[0] == 'text':
                    _, text = item
                    if text:
                        self.log(f"TEKST: {text}", 'received')
                elif item[0] == 'binary':
                    _, packet = item
                    self.process_binary_packet(packet)
                elif item[0] == 'error':
                    _, err = item
                    self.log(f"Błąd portu: {err}", 'error')
        except queue.Empty:
            pass
        self.update_data_display()
        self.update_errors()

        now = time.time() * 1000
        if now - self.last_plot_update >= self.plot_update_interval:
            self.last_plot_update = now
            self.update_plots()
        self.root.after(50, self.update_from_queue)

    def process_binary_packet(self, packet_bytes):
        data = TelemetryParser.parse_packet(packet_bytes)
        if data is None:
            self.log("Odrzucono pakiet (błędny nagłówek/CRC)", 'warning')
            return
        self.telemetry_data.update(data)
        packet_num = int(data.get('packet', 0))
        if self.last_rocket_packet == -1:
            self.last_rocket_packet = packet_num
        else:
            expected = (self.last_rocket_packet + 1) & 0xFFFF
            if packet_num != expected:
                lost = (packet_num - expected) & 0xFFFF
                self.lost_packets += lost
                self.log(f"STRATA: oczekiwano {expected}, odebrano {packet_num} (strata {lost})", 'warning')
        self.last_rocket_packet = packet_num
        self.packet_counter += 1
        self.update_packet_info()

        idx = self.packet_counter
        # Dodawanie do historii z ograniczeniem liczby punktów
        for key in self.plot_lines.keys():
            if key in data:
                val = data[key]
                if len(self.history[key]) >= self.points_limit:
                    self.history[key].popleft()
                self.history[key].append((idx, val))

        # Zapis do CSV z buforowaniem
        if self.recording and self.csv_file:
            line = f"{datetime.now().isoformat()},{packet_num},{','.join(str(data.get(f[0], '')) for f in FIELD_ORDER)}"
            self.csv_buffer.append(line)
            self.csv_buffer_size += 1
            if self.csv_buffer_size >= 50:
                self.csv_file.write("\n".join(self.csv_buffer) + "\n")
                self.csv_file.flush()
                self.csv_buffer.clear()
                self.csv_buffer_size = 0

    def update_packet_info(self):
        self.packet_info_label.config(text=f"Pakiety: {self.packet_counter} odebrane, {self.lost_packets} strat")

    def update_data_display(self):
        for name, label in self.data_labels.items():
            if name in self.telemetry_data:
                val = self.telemetry_data[name]
                if name == 'status':
                    status_map = {1: "DEBUG", 2: "FLIGHT", 3: "DUMP", 4: "SLEEP"}
                    text = status_map.get(int(val), str(val))
                elif name == 'flightStatus':
                    flight_map = {0: "IDLE", 1: "BOOST", 2: "COAST", 3: "APOGEE", 4: "DESCENT", 5: "LANDED"}
                    text = flight_map.get(int(val), str(val))
                elif name in ('drogueParachute', 'mainParachute'):
                    text = 'OPEN' if val == 1 else 'CLOSED'
                elif isinstance(val, float):
                    if name in ('time', 'battery'):
                        text = f"{val:.2f}"
                    elif name in ('gpsLat', 'gpsLng'):
                        text = f"{val:.5f}"
                    else:
                        text = f"{val:.3f}"
                else:
                    text = str(val)
                label.config(text=text)

                if name in ('drogueParachute', 'mainParachute'):
                    label.config(foreground='red' if val == 1 else 'green')
                elif name == 'battery':
                    if val >= 3.5:
                        label.config(foreground='green')
                    elif val >= 3.2:
                        label.config(foreground='gold')
                    else:
                        label.config(foreground='red')
                else:
                    label.config(foreground='black')
            else:
                label.config(text="---")

    def update_errors(self):
        error_val = self.telemetry_data.get('errorFlags', 0)
        if error_val is None:
            return
        bit_map = {
            "LORA_ERROR": 0, "LSM_ERROR": 1, "BMP1_ERROR": 2, "BMP2_ERROR": 3,
            "ADXL_ERROR": 4, "GPS_ERROR": 5, "FLASH_ERROR": 6,
            "FLASH_FILE_ERROR": 7, "MSG_TOO_LONG_ERROR": 8
        }
        for err_name, lbl in self.error_labels.items():
            is_set = (error_val >> bit_map[err_name]) & 1
            if is_set:
                lbl.config(text=f"{err_name[:-6]}: BŁĄD", foreground="red")
            else:
                lbl.config(text=f"{err_name[:-6]}: OK", foreground="green")

    def update_plots(self):
        if not self.history:
            return

        # Stałe kolory dla serii
        color_map = {
            # LSM przyspieszenia
            'lsmAccelX': 'blue', 'lsmAccelY': 'cyan', 'lsmAccelZ': 'deepskyblue',
            # ADXL przyspieszenia
            'adxlAccelX': 'orange', 'adxlAccelY': 'gold', 'adxlAccelZ': 'darkorange',
            # Filtrowane przyspieszenia
            'filteredAccelX': 'purple', 'filteredAccelY': 'violet', 'filteredAccelZ': 'magenta',
            # Prędkości
            'lsmSpeed': 'blue', 'adxlSpeed': 'orange', 'bmp1Speed': 'green', 'bmp2Speed': 'lightgreen',
            'gpsSpeed': 'red', 'filteredSpeed': 'purple',
            # Wysokości
            'lsmAlti': 'blue', 'adxlAlti': 'orange', 'bmp1Alti': 'green', 'bmp2Alti': 'lightgreen',
            'gpsAlti': 'red', 'filteredAlti': 'purple',
            # Ciśnienia
            'bmp1Press': 'darkblue', 'bmp2Press': 'steelblue',
            # Temperatury
            'lsmTemp': 'darkred', 'bmp1Temp': 'red', 'bmp2Temp': 'lightcoral',
        }

        # Usuń stare elementy z osi (ale nie czyść całej osi, bo usunie tytuły)
        for ax in [self.ax_acc, self.ax_vel, self.ax_alt, self.ax_press_temp, self.ax_temp]:
            for coll in ax.collections:
                coll.remove()
            for line in ax.lines:
                line.remove()

        # Rysuj przyspieszenie
        for prefix in ['lsm', 'adxl']:
            for axis in ['X', 'Y', 'Z']:
                key = f"{prefix}Accel{axis}"
                if key in self.history:
                    pts = list(self.history[key])
                    if pts:
                        x, y = zip(*pts)
                        color = color_map.get(key, 'gray')
                        self.ax_acc.scatter(x, y, label=f"{prefix.upper()}-{axis}", alpha=0.7, s=2, color=color)
        for axis in ['X', 'Y', 'Z']:
            key = f"filteredAccel{axis}"
            if key in self.history:
                pts = list(self.history[key])
                if pts:
                    x, y = zip(*pts)
                    color = color_map.get(key, 'purple')
                    self.ax_acc.scatter(x, y, label=f"filt-{axis}", s=2, color=color)
        self.ax_acc.legend(loc='upper left', fontsize=7)

        # Prędkość
        speed_keys = ['lsmSpeed', 'adxlSpeed', 'bmp1Speed', 'bmp2Speed', 'gpsSpeed', 'filteredSpeed']
        for key in speed_keys:
            if key in self.history:
                pts = list(self.history[key])
                if pts:
                    x, y = zip(*pts)
                    alpha = 0.7 if 'filtered' not in key else 1.0
                    color = color_map.get(key, 'gray')
                    self.ax_vel.scatter(x, y, label=key.replace('Speed', ''), alpha=alpha, s=2, color=color)
        self.ax_vel.legend(loc='upper left', fontsize=7)

        # Wysokość
        alt_keys = ['lsmAlti', 'adxlAlti', 'bmp1Alti', 'bmp2Alti', 'gpsAlti', 'filteredAlti']
        for key in alt_keys:
            if key in self.history:
                pts = list(self.history[key])
                if pts:
                    x, y = zip(*pts)
                    alpha = 0.7 if 'filtered' not in key else 1.0
                    color = color_map.get(key, 'gray')
                    self.ax_alt.scatter(x, y, label=key.replace('Alti', ''), alpha=alpha, s=2, color=color)
        self.ax_alt.legend(loc='upper left', fontsize=7)

        # Ciśnienie i temperatura
        for key in ['bmp1Press', 'bmp2Press']:
            if key in self.history:
                pts = list(self.history[key])
                if pts:
                    x, y = zip(*pts)
                    color = color_map.get(key, 'blue')
                    self.ax_press_temp.scatter(x, y, label=key, color=color, s=3)

        # Czyść oś temperatury (ale nie usuwaj etykiety i tytułu – są ustawione w setup_gui)
        self.ax_temp.clear()
        self.ax_temp.set_ylabel("°C", color='red')
        for key in ['lsmTemp', 'bmp1Temp', 'bmp2Temp']:
            if key in self.history:
                pts = list(self.history[key])
                if pts:
                    x, y = zip(*pts)
                    color = color_map.get(key, 'red')
                    self.ax_temp.scatter(x, y, label=key, color=color, s=3)

        # Legenda dla ciśnienia/temperatury
        lines1, labels1 = self.ax_press_temp.get_legend_handles_labels()
        lines2, labels2 = self.ax_temp.get_legend_handles_labels()
        self.ax_press_temp.legend(lines1 + lines2, labels1 + labels2, loc='upper left', fontsize=7)

        # Autoskalowanie
        for ax in [self.ax_acc, self.ax_vel, self.ax_alt, self.ax_press_temp]:
            ax.relim()
            ax.autoscale_view(scalex=False, scaley=True)
        self.ax_temp.relim()
        self.ax_temp.autoscale_view(scalex=False, scaley=True)

        self.canvas.draw_idle()

    def clear_plots(self):
        self.history.clear()
        self.telemetry_data.clear()
        for line in self.plot_lines.values():
            line.set_data([], [])
        self.canvas.draw_idle()
        self.log("Wykresy wyczyszczone", 'success')

    def clear_all_data(self):
        self.clear_plots()
        self.packet_counter = 0
        self.last_rocket_packet = -1
        self.lost_packets = 0
        self.update_packet_info()
        self.update_data_display()
        self.update_errors()
        self.log("Wszystkie dane zresetowane", 'success')

    def save_to_csv(self):
        if not self.history:
            messagebox.showinfo("Brak danych", "Nie ma danych do zapisu")
            return
        filename = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
        if filename:
            try:
                with open(filename, 'w', newline='') as f:
                    field_names = [f[0] for f in FIELD_ORDER]
                    f.write("timestamp,packet_number," + ",".join(field_names) + "\n")
                    indices = sorted(set(idx for lst in self.history.values() for idx, _ in lst))
                    for idx in indices:
                        row = [datetime.now().isoformat(), str(idx)]
                        for name in field_names:
                            val = None
                            if name in self.history:
                                for i, v in self.history[name]:
                                    if i == idx:
                                        val = v
                                        break
                            row.append(str(val) if val is not None else "")
                        f.write(",".join(row) + "\n")
                self.log(f"Dane zapisane do {filename}", 'success')
            except Exception as e:
                self.log(f"Błąd zapisu CSV: {e}", 'error')

    def toggle_recording(self):
        if not self.recording:
            filename = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV", "*.csv")])
            if filename:
                try:
                    self.csv_file = open(filename, 'w', newline='')
                    self.csv_buffer = []
                    self.csv_buffer_size = 0
                    self.recording = True
                    self.record_btn.config(text="Zatrzymaj zapis")
                    self.log(f"Rozpoczęto zapis do {filename}", 'success')
                except Exception as e:
                    self.log(f"Nie można otworzyć pliku: {e}", 'error')
        else:
            if self.csv_file:
                if self.csv_buffer:
                    self.csv_file.write("\n".join(self.csv_buffer) + "\n")
                    self.csv_file.flush()
                self.csv_file.close()
                self.csv_file = None
            self.recording = False
            self.record_btn.config(text="Rozpocznij zapis")
            self.log("Zapis zatrzymany", 'warning')

    def on_close(self):
        self.running = False
        self.stop_event.set()
        if self.serial_thread and self.serial_thread.is_alive():
            self.serial_thread.join(timeout=1)
        if self.csv_file:
            if self.csv_buffer:
                self.csv_file.write("\n".join(self.csv_buffer) + "\n")
            self.csv_file.close()
        self.root.destroy()


# ===========================
# URUCHOMIENIE
# ===========================
if __name__ == "__main__":
    root = tk.Tk()
    app = RocketGroundStation(root)
    root.protocol("WM_DELETE_WINDOW", app.on_close)
    root.mainloop()
