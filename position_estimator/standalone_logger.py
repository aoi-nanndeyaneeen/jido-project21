import tkinter as tk
from tkinter import ttk, messagebox
import serial
import serial.tools.list_ports
import threading
import csv
import time
import re
import os

class SerialLoggerGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("Flight Data Logger")
        self.root.geometry("400x500")
        
        self.ser = None
        self.recording = False
        self.running = True
        self.csv_file = None
        self.writer = None
        
        self.log_dir = os.path.join(os.path.dirname(__file__), "logs")
        os.makedirs(self.log_dir, exist_ok=True)
        
        self.ansi_escape = re.compile(r'\x1B(?:[@-Z\\-_]|\[[0-?]*[ -/]*[@-~])')
        self.current_data = {"Time": 0, "Roll": 0, "Pitch": 0, "Yaw": 0, "Ax": 0, "Ay": 0, "Az": 0, "Alt": 0}
        
        self.setup_ui()
        
    def setup_ui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Port Selection
        port_frame = ttk.LabelFrame(main_frame, text="Connection Info", padding="5")
        port_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(port_frame, text="Port:").grid(row=0, column=0, padx=5)
        self.port_combo = ttk.Combobox(port_frame, values=[p.device for p in serial.tools.list_ports.comports()])
        if self.port_combo['values']: self.port_combo.current(0)
        self.port_combo.grid(row=0, column=1, padx=5)
        
        self.btn_connect = ttk.Button(port_frame, text="Connect", command=self.toggle_connection)
        self.btn_connect.grid(row=0, column=2, padx=5)

        # Telemetry Display
        data_frame = ttk.LabelFrame(main_frame, text="Real-time Telemetry", padding="10")
        data_frame.pack(fill=tk.BOTH, expand=True, pady=5)
        
        self.lbl_roll = self.create_data_label(data_frame, "Roll:", 0)
        self.lbl_pitch = self.create_data_label(data_frame, "Pitch:", 1)
        self.lbl_yaw = self.create_data_label(data_frame, "Yaw:", 2)
        self.lbl_alt = self.create_data_label(data_frame, "Altitude:", 3)

        # Control
        ctrl_frame = ttk.Frame(main_frame, padding="5")
        ctrl_frame.pack(fill=tk.X, pady=10)
        
        self.btn_record = ttk.Button(ctrl_frame, text="START RECORDING", command=self.toggle_recording, state=tk.DISABLED)
        self.btn_record.pack(fill=tk.X, ipady=10)
        
        self.lbl_status = ttk.Label(main_frame, text="Status: Disconnected", foreground="gray")
        self.lbl_status.pack(side=tk.BOTTOM, anchor=tk.W)

    def create_data_label(self, parent, text, row):
        ttk.Label(parent, text=text, font=("Arial", 12)).grid(row=row, column=0, sticky=tk.W, pady=5)
        label = ttk.Label(parent, text="--.---", font=("Arial", 20, "bold"))
        label.grid(row=row, column=1, sticky=tk.E, padx=20)
        return label

    def toggle_connection(self):
        if self.ser and self.ser.is_open:
            self.disconnect()
        else:
            self.connect()

    def connect(self):
        port = self.port_combo.get()
        try:
            self.ser = serial.Serial(port, 115200, timeout=1.0)
            self.btn_connect.config(text="Disconnect")
            self.btn_record.config(state=tk.NORMAL)
            self.lbl_status.config(text=f"Status: Connected to {port}", foreground="green")
            
            self.thread = threading.Thread(target=self.serial_loop, daemon=True)
            self.thread.start()
        except Exception as e:
            messagebox.showerror("Connection Error", f"Could not open {port}.\n\nIs the Serial Monitor open?\n\n{e}")

    def disconnect(self):
        if self.recording: self.stop_recording()
        if self.ser:
            self.ser.close()
            self.ser = None
        self.btn_connect.config(text="Connect")
        self.btn_record.config(state=tk.DISABLED)
        self.lbl_status.config(text="Status: Disconnected", foreground="gray")

    def toggle_recording(self):
        if self.recording:
            self.stop_recording()
        else:
            self.start_recording()

    def start_recording(self):
        filename = os.path.join(self.log_dir, time.strftime("flight_log_%Y%m%d_%H%M%S.csv"))
        try:
            self.csv_file = open(filename, 'w', newline='', encoding='utf-8')
            self.writer = csv.writer(self.csv_file)
            self.writer.writerow(["Time", "Roll", "Pitch", "Yaw", "Ax", "Ay", "Az", "Alt"])
            
            self.recording = True
            self.start_time = time.time()
            self.btn_record.config(text="STOP RECORDING", style="Record.TButton")
            self.lbl_status.config(text=f"Status: RECORDING -> {os.path.basename(filename)}", foreground="red")
        except Exception as e:
            messagebox.showerror("File Error", f"Could not create log file: {e}")

    def stop_recording(self):
        self.recording = False
        if self.csv_file:
            self.csv_file.close()
            self.csv_file = None
        self.btn_record.config(text="START RECORDING")
        self.lbl_status.config(text="Status: Connected (Idle)", foreground="green")

    def serial_loop(self):
        while self.ser and self.ser.is_open:
            try:
                line = self.ser.readline()
                if not line: continue
                
                text = line.decode('utf-8', errors='ignore').strip()
                clean_text = self.ansi_escape.sub('', text)
                
                # Parsing logic
                if "Roll_Ang:" in clean_text:
                    m = re.search(r'Roll_Ang:\s*([+-]?[\d.]+).*Pitch_Ang:\s*([+-]?[\d.]+).*Yaw_Ang:\s*([+-]?[\d.]+)', clean_text)
                    if m:
                        self.current_data["Roll"] = float(m.group(1))
                        self.current_data["Pitch"] = float(m.group(2))
                        self.current_data["Yaw"] = float(m.group(3))
                        self.root.after(0, self.update_display)
                        
                elif "Acc: ax=" in clean_text:
                    m = re.search(r'ax=\s*([+-]?[\d.]+).*ay=\s*([+-]?[\d.]+).*az=\s*([+-]?[\d.]+)', clean_text)
                    if m:
                        self.current_data["Ax"] = float(m.group(1))
                        self.current_data["Ay"] = float(m.group(2))
                        self.current_data["Az"] = float(m.group(3))
                        
                elif "Alt:" in clean_text and "m" in clean_text:
                    m = re.search(r'Alt:\s*([+-]?[\d.]+)', clean_text)
                    if m:
                        self.current_data["Alt"] = float(m.group(1))
                        self.root.after(0, self.update_display)
                        
                        if self.recording:
                            self.current_data["Time"] = round(time.time() - self.start_time, 3)
                            self.writer.writerow([
                                self.current_data["Time"], self.current_data["Roll"], self.current_data["Pitch"], 
                                self.current_data["Yaw"], self.current_data["Ax"], self.current_data["Ay"], 
                                self.current_data["Az"], self.current_data["Alt"]
                            ])
                            self.csv_file.flush()
            except:
                break

    def update_display(self):
        self.lbl_roll.config(text=f"{self.current_data['Roll']:.2f}")
        self.lbl_pitch.config(text=f"{self.current_data['Pitch']:.2f}")
        self.lbl_yaw.config(text=f"{self.current_data['Yaw']:.2f}")
        self.lbl_alt.config(text=f"{self.current_data['Alt']:.1f} m")

if __name__ == "__main__":
    style_root = tk.Tk()
    style = ttk.Style()
    style.configure("Record.TButton", foreground="red")
    app = SerialLoggerGUI(style_root)
    style_root.mainloop()
