import pandas as pd
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import ttk, messagebox, filedialog
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import serial
import serial.tools.list_ports
import threading
import time
import re
from collections import defaultdict
import numpy as np

class SerialDataLogger:
    def __init__(self, root):
        self.root = root
        self.root.title("Serial Data Logger - PID Controller")
        self.root.geometry("1200x800")
        
        # Variables
        self.serial_port = None
        self.is_reading = False
        self.serial_thread = None
        self.data = defaultdict(list)
        self.timestamps = []
        self.keys_order = []
        self.sample_time = None
        self.discard_count = 180
        self.discard_done = False
        self.last_key_time = None
        self.key_count = 0
        
        # Configurar interfaz
        self.setup_gui()
        
    def setup_gui(self):
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configurar pesos de la grid
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(0, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Controles serial
        serial_frame = ttk.Frame(main_frame)
        serial_frame.grid(row=0, column=0, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(serial_frame, text="Puerto COM:").pack(side=tk.LEFT, padx=5)
        
        # Lista de puertos COM disponibles
        self.com_port = tk.StringVar()
        com_ports = [port.device for port in serial.tools.list_ports.comports()]
        com_ports = com_ports if com_ports else ["COM1"]
        com_dropdown = ttk.Combobox(serial_frame, textvariable=self.com_port, values=com_ports, width=10)
        com_dropdown.pack(side=tk.LEFT, padx=5)
        com_dropdown.set(com_ports[0])
        
        ttk.Label(serial_frame, text="Baudrate:").pack(side=tk.LEFT, padx=5)
        self.baudrate = tk.StringVar(value="115200")
        baud_entry = ttk.Entry(serial_frame, textvariable=self.baudrate, width=10)
        baud_entry.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(serial_frame, text="Conectar", command=self.toggle_serial)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(serial_frame, text="Exportar Excel", command=self.export_data).pack(side=tk.LEFT, padx=5)
        
        # Área de gráficos
        self.fig, self.axes = plt.subplots(3, 1, figsize=(10, 6))
        self.fig.tight_layout(pad=3.0)
        
        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.grid(row=1, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Área de texto
        text_frame = ttk.Frame(main_frame)
        text_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S), pady=(10, 0))
        
        # Añadir scrollbar al área de texto
        scrollbar = ttk.Scrollbar(text_frame)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        self.text_area = tk.Text(text_frame, height=10, yscrollcommand=scrollbar.set)
        self.text_area.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.config(command=self.text_area.yview)
        
        # Barra de estado
        self.status_var = tk.StringVar()
        self.status_var.set("Listo - Seleccione puerto COM y baudrate")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=3, column=0, sticky=(tk.W, tk.E))
        
    def toggle_serial(self):
        if self.serial_port and self.serial_port.is_open:
            self.disconnect_serial()
        else:
            self.connect_serial()
    
    def connect_serial(self):
        try:
            self.serial_port = serial.Serial(
                port=self.com_port.get(),
                baudrate=int(self.baudrate.get()),
                timeout=1,
                bytesize=serial.EIGHTBITS,
                parity=serial.PARITY_NONE,
                stopbits=serial.STOPBITS_ONE
            )
            self.is_reading = True
            self.connect_btn.config(text="Desconectar")
            self.status_var.set(f"Conectado a {self.com_port.get()} - Baudrate: {self.baudrate.get()}")
            
            # Reiniciar variables
            self.data = defaultdict(list)
            self.timestamps = []
            self.keys_order = []
            self.sample_time = None
            self.discard_done = False
            self.last_key_time = None
            self.key_count = 0
            
            # Limpiar área de texto
            self.text_area.delete(1.0, tk.END)
            
            # Iniciar hilo para lectura serial
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo conectar al puerto serial: {str(e)}")
            self.status_var.set(f"Error de conexión: {str(e)}")
    
    def disconnect_serial(self):
        self.is_reading = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connect_btn.config(text="Conectar")
        self.status_var.set("Desconectado")
    
    def read_serial_data(self):
        buffer = ""
        while self.is_reading and self.serial_port and self.serial_port.is_open:
            try:
                # Leer todos los bytes disponibles
                bytes_to_read = self.serial_port.in_waiting
                if bytes_to_read > 0:
                    data = self.serial_port.read(bytes_to_read).decode('utf-8', errors='ignore')
                    buffer += data
                    
                    # Procesar líneas completas
                    while '\n' in buffer or '\r' in buffer:
                        if '\n' in buffer:
                            line_end = buffer.find('\n')
                        else:
                            line_end = buffer.find('\r')
                            
                        line = buffer[:line_end].strip()
                        buffer = buffer[line_end + 1:]
                        
                        if line:
                            self.process_line(line)
                
                time.sleep(0.01)
            except Exception as e:
                self.status_var.set(f"Error de lectura: {str(e)}")
                if self.serial_port and self.serial_port.is_open:
                    self.serial_port.close()
                self.is_reading = False
                self.connect_btn.config(text="Conectar")
                break
    
    def process_line(self, line):
        # Buscar patrones de llave:valor para PID (Kp, Ki, Kd)
        pid_pattern = r'(Kp|Ki|Kd):\s*([-+]?\d*\.\d+|\d+)'
        matches = re.findall(pid_pattern, line)
        
        current_time = time.time()
        
        if matches:
            if not self.discard_done:
                self.key_count += 1
                if self.key_count >= self.discard_count:
                    self.discard_done = True
                    self.status_var.set(f"Descartadas {self.discard_count} muestras iniciales. Comenzando registro.")
                return
            
            for key, value in matches:
                if key not in self.keys_order:
                    self.keys_order.append(key)
                
                try:
                    numeric_value = float(value)
                    self.data[key].append(numeric_value)
                    self.timestamps.append(current_time)
                    
                    # Calcular tiempo entre muestras
                    if self.last_key_time is not None:
                        self.sample_time = (current_time - self.last_key_time) * 1000  # ms
                    
                    self.last_key_time = current_time
                except ValueError:
                    # Ignorar valores no numéricos
                    pass
            
            # Actualizar interfaz
            self.update_text_area(line)
            self.update_plots()
    
    def update_text_area(self, line):
        self.text_area.insert(tk.END, line + "\n")
        self.text_area.see(tk.END)
        
        # Limitar el número de líneas en el área de texto para evitar sobrecarga
        if int(self.text_area.index('end-1c').split('.')[0]) > 1000:
            self.text_area.delete(1.0, 2.0)
    
    def update_plots(self):
        for i, key in enumerate(self.keys_order):
            if i < len(self.axes):
                self.axes[i].clear()
                
                # Solo mostrar los últimos 100 puntos para mejor rendimiento
                display_data = self.data[key][-100:] if len(self.data[key]) > 100 else self.data[key]
                x_values = range(len(display_data))
                
                self.axes[i].plot(x_values, display_data, 'o-', markersize=3)
                self.axes[i].set_title(f"Evolución de {key}")
                self.axes[i].set_ylabel("Valor")
                self.axes[i].grid(True)
                
                # Añadir etiqueta con el valor actual
                if display_data:
                    self.axes[i].text(0.02, 0.95, f'Actual: {display_data[-1]:.4f}', 
                                     transform=self.axes[i].transAxes, verticalalignment='top',
                                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.5))
        
        # Añadir etiqueta de tiempo entre muestras si está disponible
        if self.sample_time is not None and len(self.axes) > 0:
            self.axes[0].set_xlabel(f'Tiempo entre muestras: {self.sample_time:.2f} ms')
        
        self.canvas.draw()
    
    def export_data(self):
        if not self.keys_order or len(self.data[self.keys_order[0]]) == 0:
            messagebox.showwarning("Advertencia", "No hay datos para exportar")
            return
        
        # Crear DataFrame
        df_data = {}
        max_length = max(len(self.data[key]) for key in self.keys_order)
        
        for key in self.keys_order:
            # Rellenar con NaN si las longitudes no coinciden
            padded_data = self.data[key] + [np.nan] * (max_length - len(self.data[key]))
            df_data[key] = padded_data
        
        # Agregar columna de tiempo
        if self.sample_time is not None:
            df_data['Tiempo entre muestras (ms)'] = [self.sample_time] * max_length
        
        df = pd.DataFrame(df_data)
        
        # Guardar archivo
        file_path = filedialog.asksaveasfilename(
            defaultextension=".xlsx",
            filetypes=[("Excel files", "*.xlsx"), ("CSV files", "*.csv")]
        )
        
        if file_path:
            try:
                if file_path.endswith('.xlsx'):
                    df.to_excel(file_path, index=False)
                else:
                    df.to_csv(file_path, index=False)
                    
                messagebox.showinfo("Éxito", f"Datos exportados a {file_path}")
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo exportar: {str(e)}")

if __name__ == "__main__":
    root = tk.Tk()
    app = SerialDataLogger(root)
    root.mainloop()