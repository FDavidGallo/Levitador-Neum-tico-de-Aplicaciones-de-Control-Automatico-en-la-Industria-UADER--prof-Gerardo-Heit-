import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
from scipy.optimize import curve_fit
from scipy import stats
import tkinter as tk
from tkinter import filedialog, ttk, messagebox
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import traceback
import sys
import serial
import threading
import time
from serial.tools import list_ports
from scipy.special import erf
from scipy.integrate import odeint

class AdvancedTransferFunctionAnalyzer:
    def __init__(self, root):
        self.root = root
        self.root.title("Analizador Avanzado de Función de Transferencia con Serial")
        self.root.geometry("1400x900")
        
        # Variables
        self.data = None
        self.results = None
        self.serial_port = None
        self.is_reading = False
        self.serial_thread = None
        self.consecutive_high_readings = 0  # Contador de lecturas altas consecutivas
        
        # Configurar interfaz
        self.setup_gui()
        
    def setup_gui(self):
        # Frame principal
        main_frame = ttk.Frame(self.root, padding="10")
        main_frame.grid(row=0, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        
        # Configurar pesos de la grid
        self.root.columnconfigure(0, weight=1)
        self.root.rowconfigure(0, weight=1)
        main_frame.columnconfigure(1, weight=1)
        main_frame.rowconfigure(1, weight=1)
        
        # Controles superiores
        control_frame = ttk.Frame(main_frame)
        control_frame.grid(row=0, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Button(control_frame, text="Cargar Datos CSV", command=self.load_data).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Analizar", command=self.analyze).pack(side=tk.LEFT, padx=5)
        ttk.Button(control_frame, text="Exportar Resultados", command=self.export_results).pack(side=tk.LEFT, padx=5)
        
        # Controles de comunicación serial
        serial_frame = ttk.Frame(main_frame)
        serial_frame.grid(row=1, column=0, columnspan=2, sticky=(tk.W, tk.E), pady=(0, 10))
        
        ttk.Label(serial_frame, text="Puerto COM:").pack(side=tk.LEFT, padx=5)
        self.com_port = tk.StringVar(value="COM19")
        com_entry = ttk.Entry(serial_frame, textvariable=self.com_port, width=10)
        com_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(serial_frame, text="Baudrate:").pack(side=tk.LEFT, padx=5)
        self.baudrate = tk.StringVar(value="9600")
        baud_entry = ttk.Entry(serial_frame, textvariable=self.baudrate, width=10)
        baud_entry.pack(side=tk.LEFT, padx=5)
        
        self.connect_btn = ttk.Button(serial_frame, text="Conectar", command=self.toggle_serial)
        self.connect_btn.pack(side=tk.LEFT, padx=5)
        
        ttk.Label(serial_frame, text="PWM a enviar:").pack(side=tk.LEFT, padx=5)
        self.pwm_value = tk.StringVar(value="0")
        pwm_entry = ttk.Entry(serial_frame, textvariable=self.pwm_value, width=5)
        pwm_entry.pack(side=tk.LEFT, padx=5)
        
        ttk.Button(serial_frame, text="Enviar PWM", command=self.send_pwm).pack(side=tk.LEFT, padx=5)
        ttk.Button(serial_frame, text="Barrido Automático", command=self.auto_sweep).pack(side=tk.LEFT, padx=5)
        
        # Área de gráficos
        self.fig, ((self.ax1, self.ax2), (self.ax3, self.ax4)) = plt.subplots(2, 2, figsize=(12, 8))
        self.fig.tight_layout(pad=5.0)
        
        canvas_frame = ttk.Frame(main_frame)
        canvas_frame.grid(row=2, column=0, sticky=(tk.W, tk.E, tk.N, tk.S))
        self.canvas = FigureCanvasTkAgg(self.fig, master=canvas_frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Área de resultados
        results_frame = ttk.Frame(main_frame)
        results_frame.grid(row=2, column=1, sticky=(tk.W, tk.E, tk.N, tk.S), padx=(10, 0))
        
        # Notebook para organizar resultados
        self.notebook = ttk.Notebook(results_frame)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # Pestaña de resultados
        results_tab = ttk.Frame(self.notebook)
        self.notebook.add(results_tab, text="Resultados")
        
        self.results_text = tk.Text(results_tab, wrap=tk.WORD)
        scrollbar = ttk.Scrollbar(results_tab, orient=tk.VERTICAL, command=self.results_text.yview)
        self.results_text.configure(yscrollcommand=scrollbar.set)
        
        self.results_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Pestaña de métricas
        metrics_tab = ttk.Frame(self.notebook)
        self.notebook.add(metrics_tab, text="Métricas")
        
        self.metrics_text = tk.Text(metrics_tab, wrap=tk.WORD)
        scrollbar_metrics = ttk.Scrollbar(metrics_tab, orient=tk.VERTICAL, command=self.metrics_text.yview)
        self.metrics_text.configure(yscrollcommand=scrollbar_metrics.set)
        
        self.metrics_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar_metrics.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Pestaña de datos en tiempo real
        realtime_tab = ttk.Frame(self.notebook)
        self.notebook.add(realtime_tab, text="Datos Tiempo Real")
        
        self.realtime_text = tk.Text(realtime_tab, wrap=tk.WORD, height=10)
        scrollbar_realtime = ttk.Scrollbar(realtime_tab, orient=tk.VERTICAL, command=self.realtime_text.yview)
        self.realtime_text.configure(yscrollcommand=scrollbar_realtime.set)
        
        self.realtime_text.pack(side=tk.LEFT, fill=tk.BOTH, expand=True)
        scrollbar_realtime.pack(side=tk.RIGHT, fill=tk.Y)
        
        # Barra de estado
        self.status_var = tk.StringVar()
        self.status_var.set("Listo")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief=tk.SUNKEN, anchor=tk.W)
        status_bar.grid(row=3, column=0, columnspan=2, sticky=(tk.W, tk.E))
        
        # Inicializar DataFrame para datos seriales
        self.serial_data = pd.DataFrame(columns=['time_seconds', 'pwm', 'distance'])
        self.start_time = None
        
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
                timeout=1
            )
            self.is_reading = True
            self.connect_btn.config(text="Desconectar")
            self.status_var.set(f"Conectado a {self.com_port.get()}")
            
            # Iniciar hilo para lectura serial
            self.serial_thread = threading.Thread(target=self.read_serial_data)
            self.serial_thread.daemon = True
            self.serial_thread.start()
            
            # Reiniciar datos
            self.serial_data = pd.DataFrame(columns=['time_seconds', 'pwm', 'distance'])
            self.start_time = time.time()
            self.consecutive_high_readings = 0  # Reiniciar contador de lecturas altas
            
        except Exception as e:
            messagebox.showerror("Error", f"No se pudo conectar al puerto serial: {str(e)}")
    
    def disconnect_serial(self):
        self.is_reading = False
        if self.serial_port and self.serial_port.is_open:
            self.serial_port.close()
        self.connect_btn.config(text="Conectar")
        self.status_var.set("Desconectado")
    
    def read_serial_data(self):
        while self.is_reading and self.serial_port and self.serial_port.is_open:
            try:
                if self.serial_port.in_waiting > 0:
                    line = self.serial_port.readline().decode().strip()
                    if line:
                        try:
                            distance = float(line)
                            current_time = time.time() - self.start_time
                            current_pwm = int(self.pwm_value.get()) if self.pwm_value.get().isdigit() else 0
                            
                            # Detectar saturación (más de 25 lecturas > 99.8)
                            if distance > 99.8:
                                self.consecutive_high_readings += 1
                            else:
                                self.consecutive_high_readings = 0
                            
                            # Si hay más de 25 lecturas altas consecutivas, reducir PWM
                            if self.consecutive_high_readings > 25:
                                new_pwm = max(0, current_pwm - 25)
                                self.pwm_value.set(str(new_pwm))
                                self.serial_port.write(f"{new_pwm}\n".encode())
                                self.consecutive_high_readings = 0  # Reiniciar contador
                                self.status_var.set(f"Saturación detectada. PWM reducido a {new_pwm}")
                            
                            # Agregar datos al DataFrame
                            new_data = pd.DataFrame({
                                'time_seconds': [current_time],
                                'pwm': [current_pwm],
                                'distance': [distance]
                            })
                            
                            # Asegurar que ambos DataFrames tienen las mismas columnas
                            if self.serial_data.empty:
                                self.serial_data = new_data
                            else:
                                # Verificar que las columnas coincidan
                                if list(self.serial_data.columns) == list(new_data.columns):
                                    self.serial_data = pd.concat([self.serial_data, new_data], ignore_index=True)
                                else:
                                    # Si las columnas no coinciden, reiniciar el DataFrame
                                    print("Advertencia: Estructura de columnas incompatible. Reiniciando DataFrame.")
                                    self.serial_data = new_data
                            
                            # Actualizar texto en tiempo real
                            self.realtime_text.insert(tk.END, f"T: {current_time:.2f}s, PWM: {current_pwm}, Distancia: {distance}\n")
                            self.realtime_text.see(tk.END)
                            
                        except ValueError:
                            # Ignorar líneas que no se pueden convertir a float
                            pass
            except Exception as e:
                print(f"Error en lectura serial: {str(e)}")
                print(traceback.format_exc())
                break
            time.sleep(0.01)
    
    def send_pwm(self):
        if self.serial_port and self.serial_port.is_open:
            try:
                pwm_val = int(self.pwm_value.get())
                if 0 <= pwm_val <= 255:
                    self.serial_port.write(f"{pwm_val}\n".encode())
                    self.status_var.set(f"PWM {pwm_val} enviado")
                else:
                    messagebox.showwarning("Advertencia", "El valor PWM debe estar entre 0 y 255")
            except ValueError:
                messagebox.showwarning("Advertencia", "Ingrese un valor numérico para PWM")
        else:
            messagebox.showwarning("Advertencia", "Primero conecte el puerto serial")
    
    def auto_sweep(self):
        if not (self.serial_port and self.serial_port.is_open):
            messagebox.showwarning("Advertencia", "Primero conecte el puerto serial")
            return
        
        # Barrido automático de PWM según la secuencia especificada
        def sweep_thread():
            # Escalón de 0 a 125
            self.pwm_value.set("0")
            self.serial_port.write("0\n".encode())
            self.status_var.set("PWM 0 enviado (0.5s)")
            time.sleep(0.5)
            
            self.pwm_value.set("125")
            self.serial_port.write("170\n".encode())
            self.status_var.set("PWM 125 enviado (3.5s)")
            time.sleep(3.5)
            
            # Bajar a 0 por 2 segundos
            self.pwm_value.set("0")
            self.serial_port.write("0\n".encode())
            self.status_var.set("PWM 0 enviado (2s)")
            time.sleep(2)
            
            # Subir de 0 a 100 con incrementos de 10 cada 0.25 segundos
            for pwm in range(0, 180, 10):
                if not self.is_reading:
                    break
                self.pwm_value.set(str(pwm))
                self.serial_port.write(f"{pwm}\n".encode())
                self.status_var.set(f"Barrido automático: PWM {pwm} enviado")
                time.sleep(0.25)
            for pwm in range(150, 0, -10):
                if not self.is_reading:
                    break
                self.pwm_value.set(str(pwm))
                self.serial_port.write(f"{pwm}\n".encode())
                self.status_var.set(f"Barrido automático: PWM {pwm} enviado")
                time.sleep(0.25)
            
            self.status_var.set("Barrido automático completado")
        
        threading.Thread(target=sweep_thread, daemon=True).start()
    
    def load_data(self):
        # Opción para cargar datos desde CSV o usar datos seriales
        if len(self.serial_data) > 0:
            if messagebox.askyesno("Seleccionar datos", "¿Usar datos adquiridos por serial? Si selecciona No, podrá cargar un archivo CSV."):
                self.data = self.serial_data.copy()
                self.status_var.set(f"Datos seriales cargados: {len(self.data)} registros")
                return
        
        file_path = filedialog.askopenfilename(filetypes=[("CSV files", "*.csv")])
        if file_path:
            try:
                self.data = pd.read_csv(file_path)
                self.status_var.set(f"Datos cargados: {len(self.data)} registros")
            except Exception as e:
                messagebox.showerror("Error", f"No se pudo cargar el archivo: {str(e)}")
    
    def analyze(self):
        if self.data is None:
            messagebox.showwarning("Advertencia", "Primero carga un archivo CSV o adquiere datos por serial")
            return
        
        try:
            self.status_var.set("Analizando datos...")
            self.root.update()
            
            # Realizar análisis
            self.results = self.advanced_analysis(self.data)
            
            # Mostrar resultados
            self.display_results()
            self.plot_results()
            
            self.status_var.set("Análisis completado")
            
        except Exception as e:
            messagebox.showerror("Error", f"Error en el análisis: {str(e)}")
            print(traceback.format_exc())
    
    def advanced_analysis(self, data):
        # Preprocesamiento de datos
        data_clean = data.dropna().reset_index(drop=True)
        
        # Identificar cambios de PWM
        pwm_diff = data_clean['pwm'].diff().fillna(0)
        step_indices = np.where(np.abs(pwm_diff) > 5)[0]
        
        # Añadir el último índice si no está incluido
        if len(step_indices) > 0 and step_indices[-1] < len(data_clean) - 1:
            step_indices = np.append(step_indices, len(data_clean) - 1)
        
        results = []
        
        for i in range(len(step_indices) - 1):
            start_idx = step_indices[i]
            end_idx = step_indices[i + 1]
            
            segment = data_clean.iloc[start_idx:end_idx].copy().reset_index(drop=True)
            
            if len(segment) < 20:  # Segmento demasiado corto
                continue
            
            # Preparar datos para el ajuste
            t = segment['time_seconds'] - segment['time_seconds'].iloc[0]
            y = segment['distance'].values
            u = segment['pwm'].iloc[0]  # Valor de PWM para este segmento
            
            # Valor inicial y final para normalización
            y0 = np.mean(y[:5])  # Promedio de los primeros 5 puntos
            y_ss = np.mean(y[-5:])  # Promedio de los últimos 5 puntos
            
            # Normalizar respuesta
            if (y_ss - y0) != 0:
                y_norm = (y - y0) / (y_ss - y0)
            else:
                y_norm = y - y0
            
            # Detectar zona muerta y retardo
            dead_zone, time_delay = self.detect_dead_zone_and_delay(t, y_norm)
            
            # Ajustar modelos
            try:
                # Modelo de primer orden
                def first_order(t, K, tau):
                    return K * (1 - np.exp(-t / tau))
                
                popt_1st, pcov_1st = curve_fit(first_order, t, y_norm, p0=[1, 1], maxfev=5000)
                y_pred_1st = first_order(t, *popt_1st)
                
                # Modelo de primer orden con retardo
                def first_order_with_delay(t, K, tau, L):
                    # Usar aproximación de Padé para el retardo
                    return np.where(t < L, 0, K * (1 - np.exp(-(t - L) / tau)))
                
                # Estimación inicial del retardo
                L_guess = time_delay if time_delay > 0 else 0.1
                popt_1std, pcov_1std = curve_fit(first_order_with_delay, t, y_norm, 
                                                p0=[1, 1, L_guess], 
                                                bounds=([0.1, 0.1, 0], [5, 100, t.max()/2]), 
                                                maxfev=10000)
                y_pred_1std = first_order_with_delay(t, *popt_1std)
                
                # Modelo de segundo orden
                def second_order(t, K, tau, zeta):
                    if zeta < 1:  # Subamortiguado
                        omega_n = 1 / tau
                        omega_d = omega_n * np.sqrt(1 - zeta**2)
                        return K * (1 - (np.exp(-zeta * omega_n * t) / np.sqrt(1 - zeta**2)) * 
                                  np.sin(omega_d * t + np.arccos(zeta)))
                    else:  # Sobreamortiguado
                        # Aproximación a dos polos reales
                        return K * (1 - (tau / (tau - 1)) * np.exp(-t / tau) + 
                                  (1 / (tau - 1)) * np.exp(-tau * t))
                
                popt_2nd, pcov_2nd = curve_fit(second_order, t, y_norm, p0=[1, 1, 0.7], maxfev=10000)
                y_pred_2nd = second_order(t, *popt_2nd)
                
                # Modelo de segundo orden con retardo
                def second_order_with_delay(t, K, tau, zeta, L):
                    if zeta < 1:  # Subamortiguado
                        omega_n = 1 / tau
                        omega_d = omega_n * np.sqrt(1 - zeta**2)
                        return np.where(t < L, 0, 
                                       K * (1 - (np.exp(-zeta * omega_n * (t - L)) / np.sqrt(1 - zeta**2)) * 
                                            np.sin(omega_d * (t - L) + np.arccos(zeta))))
                    else:  # Sobreamortiguado
                        # Aproximación a dos polos reales
                        return np.where(t < L, 0, 
                                       K * (1 - (tau / (tau - 1)) * np.exp(-(t - L) / tau) + 
                                            (1 / (tau - 1)) * np.exp(-tau * (t - L))))
                
                popt_2ndd, pcov_2ndd = curve_fit(second_order_with_delay, t, y_norm, 
                                                 p0=[1, 1, 0.7, L_guess], 
                                                 bounds=([0.1, 0.1, 0.1, 0], [5, 100, 5, t.max()/2]), 
                                                 maxfev=10000)
                y_pred_2ndd = second_order_with_delay(t, *popt_2ndd)
                
                # Modelo de tercer orden
                def third_order_model(t, K, tau1, tau2, tau3):
                    # Sistema de tercer orden con tres constantes de tiempo
                    def model_func(t, K, tau1, tau2, tau3):
                        # Solución analítica para sistema de tercer orden
                        A = tau2*tau3/((tau2-tau1)*(tau3-tau1))
                        B = tau1*tau3/((tau1-tau2)*(tau3-tau2))
                        C = tau1*tau2/((tau1-tau3)*(tau2-tau3))
                        return K * (1 - A*np.exp(-t/tau1) - B*np.exp(-t/tau2) - C*np.exp(-t/tau3))
                    
                    return model_func(t, K, tau1, tau2, tau3)
                
                popt_3rd, pcov_3rd = curve_fit(third_order_model, t, y_norm, 
                                              p0=[1, 1, 2, 3], 
                                              bounds=([0.1, 0.1, 0.1, 0.1], [5, 10, 10, 10]), 
                                              maxfev=10000)
                y_pred_3rd = third_order_model(t, *popt_3rd)
                
                # Calcular métricas de error para todos los modelos
                mse_1st = np.mean((y_norm - y_pred_1st) ** 2)
                mse_1std = np.mean((y_norm - y_pred_1std) ** 2)
                mse_2nd = np.mean((y_norm - y_pred_2nd) ** 2)
                mse_2ndd = np.mean((y_norm - y_pred_2ndd) ** 2)
                mse_3rd = np.mean((y_norm - y_pred_3rd) ** 2)
                
                r2_1st = 1 - np.sum((y_norm - y_pred_1st) ** 2) / np.sum((y_norm - np.mean(y_norm)) ** 2)
                r2_1std = 1 - np.sum((y_norm - y_pred_1std) ** 2) / np.sum((y_norm - np.mean(y_norm)) ** 2)
                r2_2nd = 1 - np.sum((y_norm - y_pred_2nd) ** 2) / np.sum((y_norm - np.mean(y_norm)) ** 2)
                r2_2ndd = 1 - np.sum((y_norm - y_pred_2ndd) ** 2) / np.sum((y_norm - np.mean(y_norm)) ** 2)
                r2_3rd = 1 - np.sum((y_norm - y_pred_3rd) ** 2) / np.sum((y_norm - np.mean(y_norm)) ** 2)
                
                # Determinar mejor modelo basado en AIC (Akaike Information Criterion)
                n = len(y_norm)
                aic_1st = n * np.log(mse_1st) + 2 * 2  # 2 parámetros
                aic_1std = n * np.log(mse_1std) + 2 * 3  # 3 parámetros
                aic_2nd = n * np.log(mse_2nd) + 2 * 3  # 3 parámetros
                aic_2ndd = n * np.log(mse_2ndd) + 2 * 4  # 4 parámetros
                aic_3rd = n * np.log(mse_3rd) + 2 * 4  # 4 parámetros
                
                aic_values = {
                    'Primer orden': aic_1st,
                    'Primer orden con retardo': aic_1std,
                    'Segundo orden': aic_2nd,
                    'Segundo orden con retardo': aic_2ndd,
                    'Tercer orden': aic_3rd
                }
                
                best_model = min(aic_values, key=aic_values.get)
                
                results.append({
                    'segment': i + 1,
                    'pwm': u,
                    'dead_zone': dead_zone,
                    'time_delay': time_delay,
                    'first_order': {
                        'K': popt_1st[0],
                        'tau': popt_1st[1],
                        'mse': mse_1st,
                        'r2': r2_1st,
                        'aic': aic_1st
                    },
                    'first_order_delay': {
                        'K': popt_1std[0],
                        'tau': popt_1std[1],
                        'L': popt_1std[2],
                        'mse': mse_1std,
                        'r2': r2_1std,
                        'aic': aic_1std
                    },
                    'second_order': {
                        'K': popt_2nd[0],
                        'tau': popt_2nd[1],
                        'zeta': popt_2nd[2],
                        'mse': mse_2nd,
                        'r2': r2_2nd,
                        'aic': aic_2nd
                    },
                    'second_order_delay': {
                        'K': popt_2ndd[0],
                        'tau': popt_2ndd[1],
                        'zeta': popt_2ndd[2],
                        'L': popt_2ndd[3],
                        'mse': mse_2ndd,
                        'r2': r2_2ndd,
                        'aic': aic_2ndd
                    },
                    'third_order': {
                        'K': popt_3rd[0],
                        'tau1': popt_3rd[1],
                        'tau2': popt_3rd[2],
                        'tau3': popt_3rd[3],
                        'mse': mse_3rd,
                        'r2': r2_3rd,
                        'aic': aic_3rd
                    },
                    'comparison': {
                        'best_model': best_model,
                        'aic_values': aic_values
                    },
                    'data': {
                        't': t.values,
                        'y_norm': y_norm,
                        'y_pred_1st': y_pred_1st,
                        'y_pred_1std': y_pred_1std,
                        'y_pred_2nd': y_pred_2nd,
                        'y_pred_2ndd': y_pred_2ndd,
                        'y_pred_3rd': y_pred_3rd
                    }
                })
                
            except Exception as e:
                print(f"Error en el segmento {i+1}: {str(e)}")
                continue
        
        return results
    
    def detect_dead_zone_and_delay(self, t, y_norm, threshold=0.05):
        """
        Detecta zona muerta y retardo en la respuesta del sistema.
        
        Parameters:
        t: array de tiempo
        y_norm: array de respuesta normalizada
        threshold: umbral para detectar el inicio de la respuesta (5% por defecto)
        
        Returns:
        dead_zone: valor de la zona muerta (0 si no se detecta)
        time_delay: retardo temporal detectado
        """
        # Encontrar el índice donde la respuesta supera el umbral
        response_start_idx = np.where(y_norm > threshold)[0]
        
        if len(response_start_idx) > 0:
            response_start_idx = response_start_idx[0]
            time_delay = t.iloc[response_start_idx] if response_start_idx > 0 else 0
            
            # Si hay un retardo significativo, podríamos tener zona muerta
            dead_zone = 1 if time_delay > 0.1 * t.max() else 0
        else:
            time_delay = 0
            dead_zone = 0
            
        return dead_zone, time_delay
    
    def display_results(self):
        if not self.results:
            self.results_text.delete(1.0, tk.END)
            self.results_text.insert(tk.END, "No se encontraron resultados válidos.")
            return
        
        # Contar mejores modelos
        best_model_counts = {
            'Primer orden': 0,
            'Primer orden con retardo': 0,
            'Segundo orden': 0,
            'Segundo orden con retardo': 0,
            'Tercer orden': 0
        }
        
        for r in self.results:
            best_model = r['comparison']['best_model']
            best_model_counts[best_model] += 1
        
        # Calcular promedios ponderados por calidad de ajuste para el mejor modelo general
        best_overall_model = max(best_model_counts, key=best_model_counts.get)
        
        # Calcular promedios de parámetros para el mejor modelo general
        if best_overall_model == 'Segundo orden con retardo':
            # Obtener parámetros de todos los segmentos que usan el mejor modelo
            k_values = []
            tau_values = []
            zeta_values = []
            l_values = []
            
            for r in self.results:
                if r['comparison']['best_model'] == 'Segundo orden con retardo':
                    params = r['second_order_delay']
                    k_values.append(params['K'])
                    tau_values.append(params['tau'])
                    zeta_values.append(params['zeta'])
                    l_values.append(params['L'])
            
            # Calcular promedios
            k_avg = np.mean(k_values) if k_values else 0
            tau_avg = np.mean(tau_values) if tau_values else 0
            zeta_avg = np.mean(zeta_values) if zeta_values else 0
            l_avg = np.mean(l_values) if l_values else 0
            
            # Generar texto de resultados
            result_text = "=== FUNCIÓN DE TRANSFERENCIA ESTIMADA ===\n\n"
            
            result_text += f"Mejor modelo general: {best_overall_model}\n"
            result_text += f"Función de transferencia estimada:\n"
            result_text += f"  G(s) = {k_avg:.4f} * e^(-{l_avg:.4f}s) / "
            
            if zeta_avg < 1:
                result_text += f"(s² + {2*zeta_avg:.4f}ωₙs + ωₙ²)\n"
                result_text += f"  donde ωₙ = {1/tau_avg:.4f} rad/s, ζ = {zeta_avg:.4f}\n\n"
            else:
                result_text += f"({tau_avg:.4f}s + 1)({zeta_avg:.4f}s + 1)\n\n"
        else:
            result_text = "=== FUNCIÓN DE TRANSFERENCIA ESTIMADA ===\n\n"
            result_text += f"Mejor modelo general: {best_overall_model}\n\n"
        
        result_text += "=== DISTRIBUCIÓN DE MEJORES MODELOS POR SEGMENTO ===\n\n"
        for model, count in best_model_counts.items():
            result_text += f"{model}: {count} segmentos\n"
        result_text += "\n"
        
        result_text += "=== DETALLES POR SEGMENTO ===\n\n"
        for r in self.results:
            result_text += f"Segmento {r['segment']} (PWM = {r['pwm']}):\n"
            result_text += f"  Zona muerta detectada: {'Sí' if r['dead_zone'] else 'No'}\n"
            result_text += f"  Retardo temporal: {r['time_delay']:.4f}s\n"
            result_text += f"  Mejor modelo: {r['comparison']['best_model']}\n"
            
            # Mostrar parámetros del mejor modelo
            if r['comparison']['best_model'] == 'Primer orden':
                params = r['first_order']
                result_text += f"    K={params['K']:.4f}, τ={params['tau']:.4f}, R²={params['r2']:.4f}\n"
            elif r['comparison']['best_model'] == 'Primer orden con retardo':
                params = r['first_order_delay']
                result_text += f"    K={params['K']:.4f}, τ={params['tau']:.4f}, L={params['L']:.4f}, R²={params['r2']:.4f}\n"
            elif r['comparison']['best_model'] == 'Segundo orden':
                params = r['second_order']
                result_text += f"    K={params['K']:.4f}, τ={params['tau']:.4f}, ζ={params['zeta']:.4f}, R²={params['r2']:.4f}\n"
            elif r['comparison']['best_model'] == 'Segundo orden con retardo':
                params = r['second_order_delay']
                result_text += f"    K={params['K']:.4f}, τ={params['tau']:.4f}, ζ={params['zeta']:.4f}, L={params['L']:.4f}, R²={params['r2']:.4f}\n"
            elif r['comparison']['best_model'] == 'Tercer orden':
                params = r['third_order']
                result_text += f"    K={params['K']:.4f}, τ₁={params['tau1']:.4f}, τ₂={params['tau2']:.4f}, τ₃={params['tau3']:.4f}, R²={params['r2']:.4f}\n"
            
            result_text += "\n"
        
        self.results_text.delete(1.0, tk.END)
        self.results_text.insert(tk.END, result_text)
        
        # Generar texto de métricas
        metrics_text = "=== MÉTRICAS DE AJUSTE (PROMEDIO) ===\n\n"
        
        # Calcular promedios para cada modelo
        models = ['first_order', 'first_order_delay', 'second_order', 'second_order_delay', 'third_order']
        model_names = ['Primer orden', 'Primer orden con retardo', 'Segundo orden', 
                      'Segundo orden con retardo', 'Tercer orden']
        
        for i, model in enumerate(models):
            mse_values = [r[model]['mse'] for r in self.results if model in r]
            r2_values = [r[model]['r2'] for r in self.results if model in r]
            aic_values = [r[model]['aic'] for r in self.results if model in r]
            
            if mse_values:
                avg_mse = np.mean(mse_values)
                avg_r2 = np.mean(r2_values)
                avg_aic = np.mean(aic_values)
                
                metrics_text += f"{model_names[i]}:\n"
                metrics_text += f"  MSE: {avg_mse:.6f}\n"
                metrics_text += f"  R²: {avg_r2:.4f}\n"
                metrics_text += f"  AIC: {avg_aic:.2f}\n\n"
        
        self.metrics_text.delete(1.0, tk.END)
        self.metrics_text.insert(tk.END, metrics_text)
    
    def plot_results(self):
        # Limpiar gráficos
        self.ax1.clear()
        self.ax2.clear()
        self.ax3.clear()
        self.ax4.clear()
        
        # Gráfico 1: Comparación de modelos para todos los segmentos
        colors = plt.cm.tab10(np.linspace(0, 1, len(self.results)))
        
        for i, r in enumerate(self.results):
            t = r['data']['t']
            y_norm = r['data']['y_norm']
            y_pred_best = r['data'][f'y_pred_{self.get_model_abbreviation(r["comparison"]["best_model"])}']
            
            self.ax1.plot(t, y_norm, 'o', markersize=2, alpha=0.7, color=colors[i])
            self.ax1.plot(t, y_pred_best, '-', linewidth=1, alpha=0.7, color=colors[i])
        
        self.ax1.set_xlabel('Tiempo (s)')
        self.ax1.set_ylabel('Respuesta Normalizada')
        self.ax1.set_title('Comparación de Ajustes por Segmento (Mejor Modelo)')
        self.ax1.legend(['Datos', 'Mejor Modelo'], loc='best')
        self.ax1.grid(True, alpha=0.3)
        
        # Gráfico 2: Distribución de mejores modelos
        best_model_counts = {
            'Primer orden': 0,
            'Primer orden con retardo': 0,
            'Segundo orden': 0,
            'Segundo orden con retardo': 0,
            'Tercer orden': 0
        }
        
        for r in self.results:
            best_model = r['comparison']['best_model']
            best_model_counts[best_model] += 1
        
        models = list(best_model_counts.keys())
        counts = list(best_model_counts.values())
        
        self.ax2.bar(models, counts, alpha=0.7)
        self.ax2.set_xlabel('Modelo')
        self.ax2.set_ylabel('Número de Segmentos')
        self.ax2.set_title('Distribución de Mejores Modelos')
        self.ax2.tick_params(axis='x', rotation=45)
        self.ax2.grid(True, alpha=0.3)
        
        # Gráfico 3: Comparación de R²
        r2_1st = [r['first_order']['r2'] for r in self.results]
        r2_1std = [r['first_order_delay']['r2'] for r in self.results]
        r2_2nd = [r['second_order']['r2'] for r in self.results]
        r2_2ndd = [r['second_order_delay']['r2'] for r in self.results]
        r2_3rd = [r['third_order']['r2'] for r in self.results]
        
        segments = range(1, len(self.results) + 1)
        
        self.ax3.plot(segments, r2_1st, 'o-', label='1er Orden', alpha=0.7)
        self.ax3.plot(segments, r2_1std, 's-', label='1er Orden con retardo', alpha=0.7)
        self.ax3.plot(segments, r2_2nd, '^-', label='2do Orden', alpha=0.7)
        self.ax3.plot(segments, r2_2ndd, 'd-', label='2do Orden con retardo', alpha=0.7)
        self.ax3.plot(segments, r2_3rd, 'v-', label='3er Orden', alpha=0.7)
        
        self.ax3.set_xlabel('Segmento')
        self.ax3.set_ylabel('R²')
        self.ax3.set_title('Comparación de Calidad de Ajuste (R²)')
        self.ax3.legend(loc='best')
        self.ax3.grid(True, alpha=0.3)
        
        # Gráfico 4: Parámetros de retardo y zona muerta
        time_delays = [r['time_delay'] for r in self.results]
        dead_zones = [r['dead_zone'] for r in self.results]
        
        self.ax4.plot(segments, time_delays, 'o-', label='Retardo (s)')
        self.ax4.plot(segments, dead_zones, 's-', label='Zona muerta (0/1)')
        self.ax4.set_xlabel('Segmento')
        self.ax4.set_ylabel('Valor')
        self.ax4.set_title('Retardo y Zona Muerta por Segmento')
        self.ax4.legend()
        self.ax4.grid(True, alpha=0.3)
        
        # Ajustar diseño y dibujar
        self.fig.tight_layout()
        self.canvas.draw()
    
    def get_model_abbreviation(self, model_name):
        """Convierte el nombre del modelo a su abreviatura para acceder a los datos."""
        abbreviations = {
            'Primer orden': '1st',
            'Primer orden con retardo': '1std',
            'Segundo orden': '2nd',
            'Segundo orden con retardo': '2ndd',
            'Tercer orden': '3rd'
        }
        return abbreviations.get(model_name, '1st')
    
    def export_results(self):
        if not self.results:
            messagebox.showwarning("Advertencia", "No hay resultados para exportar")
            return
        
        file_path = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV files", "*.csv"), ("Text files", "*.txt"), ("All files", "*.*")]
        )
        
        if file_path:
            try:
                # Crear DataFrame con resultados
                export_data = []
                for r in self.results:
                    row = {
                        'segmento': r['segment'],
                        'pwm': r['pwm'],
                        'zona_muerta': r['dead_zone'],
                        'retardo_temporal': r['time_delay'],
                        'mejor_modelo': r['comparison']['best_model'],
                    }
                    
                    # Agregar parámetros de todos los modelos
                    models = ['first_order', 'first_order_delay', 'second_order', 'second_order_delay', 'third_order']
                    model_names = ['1er_orden', '1er_orden_retardo', '2do_orden', '2do_orden_retardo', '3er_orden']
                    
                    for i, model in enumerate(models):
                        prefix = model_names[i]
                        if model in r:
                            params = r[model]
                            for key, value in params.items():
                                if key != 'aic':  # Excluir AIC para simplificar
                                    row[f'{prefix}_{key}'] = value
                    
                    export_data.append(row)
                
                df = pd.DataFrame(export_data)
                df.to_csv(file_path, index=False)
                
                self.status_var.set(f"Resultados exportados a {file_path}")
                
            except Exception as e:
                messagebox.showerror("Error", f"No se pudieron exportar los resultados: {str(e)}")

def main():
    root = tk.Tk()
    app = AdvancedTransferFunctionAnalyzer(root)
    root.mainloop()

if __name__ == "__main__":
    main()