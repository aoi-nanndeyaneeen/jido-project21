import serial, threading
import customtkinter as ctk
from matplotlib.figure import Figure
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg

PORT = "COM3"  # 環境に合わせて変更してください
BAUD = 115200
MAX_DATA = 50

class ServoBlock(ctk.CTkFrame):
    def __init__(self, master, name, val, v_min, v_max, offset, send_func):
        super().__init__(master)
        self.name = name
        self.send_func = send_func
        
        # --- 上段: タイトルと設定値（Min/Max/Offset）の表示 ---
        info_text = f"Min: {v_min} | Max: {v_max} | Offset: {offset}"
        self.title_lbl = ctk.CTkLabel(self, text=f"{name}  ({info_text})", font=("Arial", 12, "bold"))
        self.title_lbl.pack(anchor="w", padx=10, pady=(5, 0))
        
        # --- 中段: スライダーとテキスト入力（キーボード用） ---
        ctrl_frame = ctk.CTkFrame(self, fg_color="transparent")
        ctrl_frame.pack(fill="x", padx=10, pady=5)
        
        # スライダー
        self.slider = ctk.CTkSlider(ctrl_frame, from_=v_min, to=v_max, command=self.on_slider)
        self.slider.set(float(val))
        self.slider.pack(side="left", fill="x", expand=True, padx=(0, 10))
        
        # テキスト入力ボックス (キーボード入力用)
        self.entry = ctk.CTkEntry(ctrl_frame, width=60)
        self.entry.insert(0, str(val))
        self.entry.pack(side="right")
        
        # Enterキーを押した瞬間に on_enter を実行する魔法
        self.entry.bind("<Return>", self.on_enter)
        
        # --- 下段: 現在位置の表示 ---
        self.current_lbl = ctk.CTkLabel(self, text=f"現在位置: {val}", text_color="#3B8ED0")
        self.current_lbl.pack(anchor="e", padx=10, pady=(0, 5))

    def on_slider(self, v):
        """マウスでスライダーを動かしたときの処理"""
        # テキストボックスの数値をスライダーに合わせる
        self.entry.delete(0, "end")
        self.entry.insert(0, f"{float(v):.1f}")
        # Arduinoへ送信
        self.send_func(self.name, v)

    def on_enter(self, event):
        """キーボードで数値を打ち込み、Enterを押したときの処理"""
        try:
            v = float(self.entry.get())
            # スライダーの位置を入力された数値に合わせる
            self.slider.set(v)
            # Arduinoへ送信
            self.send_func(self.name, v)
        except ValueError:
            pass # "abc"のような数字以外の入力は無視してエラーを防ぐ

    def update_current_pos(self, current_val):
        """Arduinoから現在位置が届いたときに呼ばれる処理"""
        self.current_lbl.configure(text=f"現在位置: {current_val}")

class App(ctk.CTk):
    def __init__(self):
        super().__init__()
        self.title("Universal Arduino Debugger")
        self.geometry("900x600")
        
        # --- レイアウト構築 ---
        # 左側: スライダー用スクロールエリア
        self.scroll = ctk.CTkScrollableFrame(self, width=250)
        self.scroll.pack(side="left", fill="y", padx=10, pady=10)
        
        # 右側: グラフ＆ログ用のコンテナ
        right_frame = ctk.CTkFrame(self)
        right_frame.pack(side="right", fill="both", expand=True, padx=10, pady=10)
        
        # 右側 上部: グラフエリア
        self.fig = Figure(figsize=(5, 3), dpi=100)
        self.ax = self.fig.add_subplot(111)
        self.canvas = FigureCanvasTkAgg(self.fig, master=right_frame)
        self.canvas.get_tk_widget().pack(fill="both", expand=True, pady=(0, 5))
        
        # 右側 下部: ログエリア
        self.log_box = ctk.CTkTextbox(right_frame, height=120, font=("Consolas", 12))
        self.log_box.pack(fill="x")
        self.log_box.insert("end", "--- System Ready ---\n")

        # 変数管理
        self.params = {}
        self.plot_data = {}

        # シリアル接続
        try:
            self.ser = serial.Serial(PORT, BAUD, timeout=0.1)
            self.log("[SYS] Serial Connected!")
        except Exception as e:
            self.ser = None
            self.log(f"[SYS] Serial Offline: {e}")

        # 受信スレッド開始
        threading.Thread(target=self.recv_loop, daemon=True).start()

    def log(self, text):
        """ログを下部に追加し、自動スクロール"""
        self.log_box.insert("end", text + "\n")
        self.log_box.see("end")
        if int(self.log_box.index('end-1c').split('.')[0]) > 500:
            self.log_box.delete("1.0", "2.0")

    def update_graph(self, name, val):
        """グラフの更新"""
        if name not in self.plot_data:
            self.plot_data[name] = []
            
        self.plot_data[name].append(float(val))
        if len(self.plot_data[name]) > MAX_DATA:
            self.plot_data[name].pop(0)
            
        self.ax.clear()
        for label, values in self.plot_data.items():
            self.ax.plot(values, label=label)
        self.ax.legend(loc="upper left")
        self.canvas.draw()

    def add_ui(self, name, val, v_min, v_max):
        """スライダーの動的追加"""
        if name in self.params: return
        frame = ctk.CTkFrame(self.scroll); frame.pack(fill="x", pady=5)
        lbl = ctk.CTkLabel(frame, text=f"{name}: {val}"); lbl.pack()

        def send_val(v):
            lbl.configure(text=f"{name}: {float(v):.2f}")
            msg = f"{name}:{v}"
            self.log(f"[SEND] {msg}")
            if self.ser: self.ser.write((msg + "\n").encode())

        slider = ctk.CTkSlider(frame, from_=v_min, to=v_max, command=send_val)
        slider.set(float(val)); slider.pack(fill="x", padx=10, pady=5)
        self.params[name] = slider

    def recv_loop(self):
        """シリアル受信ループ"""
        while True:
            if not self.ser: continue
            try:
                line = self.ser.readline().decode('utf-8', errors='ignore').strip()
                if not line: continue
                
                self.after(0, self.log, f"[RECV] {line}")
                
                if line.startswith("INIT:"):
                    _, name, val, v_min, v_max = line.split(":")
                    self.after(0, self.add_ui, name, val, float(v_min), float(v_max))
                elif ":" in line:
                    name, val = line.split(":")
                    self.after(0, self.update_graph, name, val)
            except Exception as e:
                pass

if __name__ == "__main__":
    app = App()
    app.mainloop()