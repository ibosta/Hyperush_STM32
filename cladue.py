import sys
import socket
import json
import threading
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QPushButton, QLabel, QVBoxLayout, QHBoxLayout, QFrame, QGridLayout, QGraphicsDropShadowEffect
)
from PyQt5.QtCore import Qt, QUrl, QRect, QTimer, pyqtSignal, QObject
from PyQt5.QtGui import QFont, QColor, QPainter, QLinearGradient, QPen, QBrush
from PyQt5.QtWebEngineWidgets import QWebEngineView

class ModernButton(QPushButton):
    def __init__(self, text, color='#00ffe7', border_color='#00ffe7', bg_color='#181818', hover_color='#222', font_size=13, danger=False):
        super().__init__(text)
        self.default_style = f'''
            QPushButton {{
                color: {color};
                border: 2px solid {border_color};
                border-radius: 10px;
                padding: 10px 0px;
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 {bg_color}, stop:1 #23272b);
                font-size: {font_size}px;
                font-weight: bold;
                letter-spacing: 1px;
            }}
            QPushButton:hover {{
                background: {hover_color};
                color: #fff;
            }}
        '''
        if danger:
            self.default_style = self.default_style.replace(color, '#fff').replace(border_color, '#ff0000')
        self.setStyleSheet(self.default_style)
        self.setFont(QFont("Segoe UI", font_size, QFont.Bold))
        self.setFixedHeight(48)
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(12)
        shadow.setColor(QColor(0, 255, 231, 80))
        shadow.setOffset(0, 2)
        self.setGraphicsEffect(shadow)

class TunnelPositionBar(QWidget):
    def __init__(self, min_pos=0, max_pos=100, sections=4, parent=None):
        super().__init__(parent)
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.sections = sections
        self.position = min_pos
        self.setMinimumHeight(60)
        self.setMaximumHeight(70)
        self.setStyleSheet("background: transparent;")

    def set_position(self, pos):
        self.position = max(self.min_pos, min(self.max_pos, pos))
        print(f"Position Bar Updated - Position: {self.position}m")  # Debug
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect().adjusted(2, 12, -2, -8)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(QPen(QColor('#00ffe7'), 2))
        painter.setBrush(QColor(24, 24, 24))
        painter.drawRoundedRect(rect, 12, 12)

        bar_margin = 58
        bar_rect = QRect(rect.left()+bar_margin, rect.top()+10, rect.width()-2*bar_margin, 18)
        painter.setPen(QPen(QColor('#00ffe7'), 1))
        painter.setBrush(QColor(30, 30, 30))
        painter.drawRoundedRect(bar_rect, 8, 8)

        rel_pos = (self.position - self.min_pos) / (self.max_pos - self.min_pos)
        fill_width = int(bar_rect.width() * rel_pos)
        if fill_width > 0:
            grad = QLinearGradient(bar_rect.left(), 0, bar_rect.left()+fill_width, 0)
            grad.setColorAt(0.0, QColor('#00ff00'))
            grad.setColorAt(0.5, QColor('#ffe700'))
            grad.setColorAt(1.0, QColor('#ff0000'))
            painter.setBrush(QBrush(grad))
            painter.setPen(Qt.NoPen)
            painter.drawRoundedRect(QRect(bar_rect.left(), bar_rect.top(), fill_width, bar_rect.height()), 8, 8)

        painter.setPen(QPen(QColor('#00ffe7'), 1, Qt.DashLine))
        for i in range(1, self.sections):
            x = bar_rect.left() + i * bar_rect.width() / self.sections
            painter.drawLine(int(x), bar_rect.top(), int(x), bar_rect.bottom())

        point_x = bar_rect.left() + rel_pos * bar_rect.width()
        painter.setPen(QPen(QColor('#fff'), 1))
        painter.setBrush(QBrush(QColor('#fff')))
        painter.drawEllipse(int(point_x)-7, bar_rect.center().y()-7, 14, 14)

        painter.setFont(QFont("Segoe UI", 8, QFont.Bold))
        painter.setPen(QColor('#fff'))
        painter.drawText(bar_rect.left()-32, bar_rect.center().y()+6, "0m")
        painter.drawText(bar_rect.right()+8, bar_rect.center().y()+6, "100m")

        painter.setFont(QFont("Orbitron", 10, QFont.Bold))
        painter.setPen(QColor('#00ffe7'))
        painter.drawText(rect.left(), rect.top()-12, rect.width(), 16, Qt.AlignCenter, "ARAÇ TÜNEL POZİSYONU")

class UDPDataReceiver(QObject):
    data_received = pyqtSignal(dict)
    connection_status_changed = pyqtSignal(str)

    def __init__(self, host='0.0.0.0', port=8888):
        super().__init__()
        self.host = host
        self.port = port
        self.socket = None
        self.running = False
        self.thread = None
        self.last_data_time = 0

    def start_listening(self):
        if self.running:
            return
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.socket.bind((self.host, self.port))
            self.socket.settimeout(1.0)
            self.running = True
            self.thread = threading.Thread(target=self._listen_loop, daemon=True)
            self.thread.start()
            print(f"UDP sunucu {self.host}:{self.port} adresinde dinlemeye başladı")
        except Exception as e:
            print(f"UDP sunucu başlatma hatası: {e}")

    def stop_listening(self):
        self.running = False
        if self.socket:
            self.socket.close()
        if self.thread:
            self.thread.join(timeout=2)

    def _listen_loop(self):
        import time
        while self.running:
            try:
                print("UDP paket bekleniyor...")
                data, addr = self.socket.recvfrom(4096)
                print(f"UDP paket alındı: {len(data)} byte, gönderen: {addr}")
                try:
                    decoded_data = data.decode('utf-8')
                    start = decoded_data.find('{')
                    end = decoded_data.rfind('}') + 1
                    if start == -1 or end == 0:
                        print("JSON verisi bulunamadı")
                        continue
                    json_data = decoded_data[start:end]
                    print(f"Çözümlenen veri: {json_data}")
                    try:
                        sensor_data = json.loads(json_data)
                        if not isinstance(sensor_data, dict):
                            print("Geçersiz JSON formatı (dictionary değil)")
                            continue
                    except json.JSONDecodeError as e:
                        print(f"JSON parse hatası: {e}")
                        continue
                    for key in sensor_data:
                        if sensor_data[key] == "":
                            if key in ['ir_state', 'ir_state2']:
                                sensor_data[key] = 0
                            else:
                                sensor_data[key] = "0.0"
                except UnicodeDecodeError as e:
                    print(f"Veri çözümleme hatası: {e}")
                    continue
                self.last_data_time = time.time()
                required_fields = ['temperature', 'object_temp', 'accel_x', 'accel_y', 
                                'accel_z', 'gyro_x', 'gyro_y', 'gyro_z', 
                                'total_distance', 'encoder_speed', 'ir_state', 'ir_state2', 'current']
                if not all(field in sensor_data for field in required_fields):
                    missing_fields = [field for field in required_fields if field not in sensor_data]
                    print(f"Eksik alanlar: {missing_fields}")
                    continue
                print(f"JSON başarıyla parse edildi: {list(sensor_data.keys())}")
                print(f"Sensör verileri: {sensor_data}")
                print("Veri UI'a gönderiliyor:", sensor_data)
                self.data_received.emit(sensor_data)
                self.connection_status_changed.emit("Başarılı")
            except socket.timeout:
                print("Socket timeout - devam ediliyor...")
                import time
                if time.time() - self.last_data_time > 5.0:
                    self.connection_status_changed.emit("Bağlantı Kesildi")
                continue
            except OSError as e:
                if self.running:
                    print(f"Socket hatası: {e}")
                break
            except Exception as e:
                if self.running:
                    print(f"UDP veri alma hatası: {e}")
                break

class MainWindow(QMainWindow):
    def closeEvent(self, event):
        if hasattr(self, 'udp_receiver'):
            self.udp_receiver.stop_listening()
        event.accept()

    def __init__(self):
        super().__init__()
        self.setWindowTitle("AXAR KONTROL PANELİ")
        self.setGeometry(100, 100, 1400, 800)
        self.setStyleSheet("background-color: qlineargradient(x1:0, y1:0, x2:1, y2:1, stop:0 #101010, stop:1 #181c20);")

        self.sensor_data = {
            'temperature': 0.0,
            'object_temp': 0.0,
            'accel_x': 0.0,
            'accel_y': 0.0,
            'accel_z': 0.0,
            'gyro_x': 0.0,
            'gyro_y': 0.0,
            'gyro_z': 0.0,
            'encoder_pos': 0.0,
            'encoder_speed': 0.0,
            'ir_state': 0,
            'ir_state2': 0,
            'current': 0.0,
            'connection_status': 'Bağlantı Bekleniyor'
        }

        self.temp_label = None
        self.pos_label = None
        self.speed_label = None
        self.acc_label = None
        self.orient_label = None
        self.power_label = None
        self.status_labels = {}
        self.tunnel_bar = None

        self.update_timer = QTimer()
        self.update_timer.timeout.connect(self.update_ui_labels)
        self.update_timer.start(100)

        try:
            self.udp_receiver = UDPDataReceiver(host='0.0.0.0', port=8080)
            self.udp_receiver.data_received.connect(self.update_sensor_data)
            self.udp_receiver.connection_status_changed.connect(self.update_connection_status)

            self.udp_sender = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            self.stm32_address = ('192.168.1.20', 8080)

            print(f"UDP alıcı başlatıldı: 0.0.0.0:8080")
            print(f"UDP gönderici hazır, hedef: {self.stm32_address}")
        except Exception as e:
            print(f"UDP başlatma hatası: {e}")
            import traceback
            traceback.print_exc()

        self.initUI()

        self.udp_receiver.start_listening()

    def panel_shadow(self, widget, color='#00ffe7'):
        shadow = QGraphicsDropShadowEffect()
        shadow.setBlurRadius(24)
        shadow.setColor(QColor(color))
        shadow.setOffset(0, 0)
        widget.setGraphicsEffect(shadow)

    def initUI(self):
        main_widget = QWidget()
        main_layout = QVBoxLayout()

        top_layout = QHBoxLayout()
        center_layout = QHBoxLayout()
        axar_label = QLabel("AXAR KONTROL PANELİ")
        axar_label.setFont(QFont("Orbitron", 11, QFont.Bold))
        axar_label.setStyleSheet("color: #00ffe7; border: 2px solid #00ffe7; border-radius: 10px; padding: 4px 8px; background: #181818; letter-spacing: 1px;")
        axar_label.setAlignment(Qt.AlignCenter)
        axar_label.setFixedWidth(320)
        self.panel_shadow(axar_label, '#00ffe7')

        hyperrush_label = QLabel("HYPERUSH")
        hyperrush_label.setFont(QFont("Orbitron", 12, QFont.Bold))
        hyperrush_label.setStyleSheet("color: #fff; border: 2px solid #00ffe7; border-radius: 10px; padding: 5px 12px; background: #181818; letter-spacing: 1px;")
        hyperrush_label.setAlignment(Qt.AlignCenter)
        hyperrush_label.setFixedWidth(200)
        self.panel_shadow(hyperrush_label, '#00ffe7')

        self.tunnel_bar = TunnelPositionBar(min_pos=0, max_pos=30)
        from PyQt5.QtWidgets import QSizePolicy
        self.tunnel_bar.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Fixed)

        top_layout.addWidget(axar_label)
        top_layout.addSpacing(18)
        top_layout.addWidget(self.tunnel_bar)
        top_layout.addSpacing(18)
        top_layout.addWidget(hyperrush_label)
        main_layout.addLayout(top_layout)

        bottom_panel = QFrame()
        bottom_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 14px; background: #181818;")
        self.panel_shadow(bottom_panel, '#00ffe7')
        bottom_layout2 = QGridLayout()
        bottom_layout2.setHorizontalSpacing(8)
        bottom_layout2.setVerticalSpacing(4)

        temp_panel = QFrame()
        temp_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 8px; background: #181818;")
        temp_layout = QVBoxLayout()
        temp_layout.setAlignment(Qt.AlignCenter)
        temp_title = QLabel("\U0001F321 Sıcaklık")
        temp_title.setFont(QFont("Orbitron", 9, QFont.Bold))
        temp_title.setStyleSheet("color: #00ffe7; padding-bottom: 2px;")
        temp_title.setAlignment(Qt.AlignHCenter)
        temp_layout.addWidget(temp_title)
        self.temp_label = QLabel("Kapsül: 22.0°C\nNesne: 20.0°C")
        self.temp_label.setFont(QFont("Orbitron", 10, QFont.Bold))
        self.temp_label.setStyleSheet("color: #00ffe7; padding: 2px 0px;")
        self.temp_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        temp_layout.addWidget(self.temp_label)
        temp_panel.setLayout(temp_layout)

        pos_panel = QFrame()
        pos_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 8px; background: #181818;")
        pos_layout = QVBoxLayout()
        pos_layout.setAlignment(Qt.AlignCenter)
        pos_title = QLabel("\U0001F4CD Pozisyon")
        pos_title.setFont(QFont("Orbitron", 9, QFont.Bold))
        pos_title.setStyleSheet("color: #00ffe7; padding-bottom: 2px;")
        pos_title.setAlignment(Qt.AlignHCenter)
        pos_layout.addWidget(pos_title)
        self.pos_label = QLabel("X: 0.00 m")
        self.pos_label.setFont(QFont("Orbitron", 10, QFont.Bold))
        self.pos_label.setStyleSheet("color: #00ffe7; padding: 2px 0px;")
        self.pos_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        pos_layout.addWidget(self.pos_label)
        pos_panel.setLayout(pos_layout)

        speed_panel = QFrame()
        speed_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 8px; background: #181818;")
        speed_layout = QVBoxLayout()
        speed_layout.setAlignment(Qt.AlignCenter)
        speed_title = QLabel("\U0001F680 Hız")
        speed_title.setFont(QFont("Orbitron", 9, QFont.Bold))
        speed_title.setStyleSheet("color: #00ffe7; padding-bottom: 2px;")
        speed_title.setAlignment(Qt.AlignHCenter)
        speed_layout.addWidget(speed_title)
        self.speed_label = QLabel("X: 0.0 m/s")
        self.speed_label.setFont(QFont("Orbitron", 10, QFont.Bold))
        self.speed_label.setStyleSheet("color: #00ffe7; padding: 2px 0px;")
        self.speed_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        speed_layout.addWidget(self.speed_label)
        speed_panel.setLayout(speed_layout)

        acc_panel = QFrame()
        acc_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 8px; background: #181818;")
        acc_layout = QVBoxLayout()
        acc_layout.setAlignment(Qt.AlignCenter)
        acc_title = QLabel("\u26A1 İvme")
        acc_title.setFont(QFont("Orbitron", 9, QFont.Bold))
        acc_title.setStyleSheet("color: #00ffe7; padding-bottom: 2px;")
        acc_title.setAlignment(Qt.AlignHCenter)
        acc_layout.addWidget(acc_title)
        self.acc_label = QLabel("X: 0.00 m/s²\nY: 0.00 m/s²\nZ: 0.00 m/s²")
        self.acc_label.setFont(QFont("Orbitron", 10, QFont.Bold))
        self.acc_label.setStyleSheet("color: #00ffe7; padding: 2px 0px;")
        self.acc_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        acc_layout.addWidget(self.acc_label)
        acc_panel.setLayout(acc_layout)

        orient_panel = QFrame()
        orient_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 8px; background: #181818;")
        orient_layout = QVBoxLayout()
        orient_layout.setAlignment(Qt.AlignCenter)
        orient_title = QLabel("\U0001F9BE Yönelim")
        orient_title.setFont(QFont("Orbitron", 9, QFont.Bold))
        orient_title.setStyleSheet("color: #00ffe7; padding-bottom: 2px;")
        orient_title.setAlignment(Qt.AlignHCenter)
        orient_layout.addWidget(orient_title)
        self.orient_label = QLabel("X: 0.00°\nY: 0.00°\nZ: 0.00°")
        self.orient_label.setFont(QFont("Orbitron", 10, QFont.Bold))
        self.orient_label.setStyleSheet("color: #00ffe7; padding: 2px 0px;")
        self.orient_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        orient_layout.addWidget(self.orient_label)
        orient_panel.setLayout(orient_layout)

        power_panel = QFrame()
        power_panel.setStyleSheet("border: 2px solid #ff0000; border-radius: 8px; background: #181818;")
        power_layout = QVBoxLayout()
        power_layout.setAlignment(Qt.AlignCenter)
        power_title = QLabel("\u26A1 Güç Tüketimi")
        power_title.setFont(QFont("Orbitron", 9, QFont.Bold))
        power_title.setStyleSheet("color: #ff0000; padding-bottom: 2px;")
        power_title.setAlignment(Qt.AlignHCenter)
        power_layout.addWidget(power_title)
        self.power_label = QLabel("0 W")
        self.power_label.setFont(QFont("Orbitron", 10, QFont.Bold))
        self.power_label.setStyleSheet("color: #ff0000; padding: 2px 0px;")
        self.power_label.setAlignment(Qt.AlignHCenter | Qt.AlignVCenter)
        power_layout.addWidget(self.power_label)
        power_panel.setLayout(power_layout)

        bottom_layout2.addWidget(temp_panel, 0, 0)
        bottom_layout2.addWidget(pos_panel, 0, 1)
        bottom_layout2.addWidget(speed_panel, 0, 2)
        bottom_layout2.addWidget(acc_panel, 0, 3)
        bottom_layout2.addWidget(orient_panel, 0, 4)
        bottom_layout2.addWidget(power_panel, 0, 5)
        bottom_panel.setLayout(bottom_layout2)

        left_panel = QFrame()
        left_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 18px; background: #181818;")
        left_panel.setFixedWidth(340)
        self.panel_shadow(left_panel, '#00ffe7')
        from PyQt5.QtWidgets import QSizePolicy
        left_layout = QVBoxLayout()
        left_layout.setContentsMargins(18, 18, 18, 18)
        left_layout.setSpacing(0)
        left_title = QLabel("KONTROL PANELİ")
        left_title.setFont(QFont("Orbitron", 12, QFont.Bold))
        left_title.setStyleSheet(
            "color: #ffe700; border: 2px solid #00ffe7; border-radius: 10px; "
            "padding: 10px 0px; background: #181818; letter-spacing: 1.5px; margin-bottom: 12px; font-weight: 600;"
        )
        left_title.setAlignment(Qt.AlignCenter)
        left_title.setFixedHeight(58)
        left_layout.addWidget(left_title)
        left_layout.addStretch(1)
        button_names = [
            ("SİSTEM BAŞLAT", False),
            ("LEVİTASYON BAŞLAT", False),
            ("LEVİTASYON DURDUR", False),
            ("ACİL DURUM", True),
            ("GERİ AL", False),
            ("İLERİ AL", False),
            ("ARACI YAVAŞLAT", False),
            ("AKTÜATÖR YUKARI", False),
            ("AKTÜATÖR AŞAĞI", False),
            ("AKTÜATÖR DURDUR", False),
        ]
        button_widgets = []
        for btn_text, is_danger in button_names:
            btn = ModernButton(
                btn_text,
                font_size=15,
                color="#fff" if is_danger else "#00ffe7",
                border_color="#ff0000" if is_danger else "#00ffe7",
                danger=is_danger
            )
            btn.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
            if btn_text == "SİSTEM BAŞLAT":
                btn.clicked.connect(lambda: self.send_command("sistem_baslat"))
            elif btn_text == "LEVİTASYON BAŞLAT":
                btn.clicked.connect(lambda: self.send_command("levitasyon_baslat"))
            elif btn_text == "LEVİTASYON DURDUR":
                btn.clicked.connect(lambda: self.send_command("levitasyon_durdur"))
            elif btn_text == "ACİL DURUM":
                btn.clicked.connect(lambda: self.send_command("acil_durum"))
            elif btn_text == "GERİ AL":
                btn.clicked.connect(lambda: self.send_command("ileri_al"))
            elif btn_text == "İLERİ AL":
                btn.clicked.connect(lambda: self.send_command("geri_al"))
            elif btn_text == "ARACI YAVAŞLAT":
                btn.clicked.connect(lambda: self.send_command("yavasla"))
            elif btn_text == "AKTÜATÖR YUKARI":
                btn.clicked.connect(lambda: self.send_command("aktuator_yukari"))
            elif btn_text == "AKTÜATÖR AŞAĞI":
                btn.clicked.connect(lambda: self.send_command("aktuator_asagi"))
            elif btn_text == "AKTÜATÖR DURDUR":
                btn.clicked.connect(lambda: self.send_command("aktuator_durdur"))
            button_widgets.append(btn)
        n = len(button_widgets)
        for i, btn in enumerate(button_widgets):
            left_layout.addWidget(btn)
            if i < n - 1:
                left_layout.addStretch(1)
        left_layout.addStretch(1)
        left_panel.setLayout(left_layout)

        right_panel = QFrame()
        right_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 18px; background: #181818;")
        right_panel.setFixedWidth(340)
        self.panel_shadow(right_panel, '#00ffe7')
        right_layout = QVBoxLayout()
        right_title = QLabel("⚠️ SİSTEM DURUMU")
        right_title.setFont(QFont("Orbitron", 13, QFont.Bold))
        right_title.setStyleSheet("color: #ffe700; margin-bottom: 10px; letter-spacing: 1px;")
        right_title.setAlignment(Qt.AlignCenter)
        right_layout.addWidget(right_title)
        status_labels = [
            ("Bağlantı Durumu:", "Bağlantı Bekleniyor", "#ff0000"),
            ("Sıcaklık Durumu:", "Normal", "#00ff00"),
            ("Güç Durumu:", "Normal", "#00ff00"),
            ("Araç Durumu:", "Hazır", "#00ffe7"),
        ]
        for title, value, color in status_labels:
            row = QHBoxLayout()
            label = QLabel(title)
            label.setFont(QFont("Orbitron", 9, QFont.Bold))
            label.setStyleSheet("color: #00ffe7;")
            label.setFixedHeight(32)
            label.setMinimumWidth(150)
            label.setAlignment(Qt.AlignVCenter | Qt.AlignRight)
            value_label = QLabel(value)
            value_label.setFont(QFont("Orbitron", 9, QFont.Bold))
            value_label.setStyleSheet(f"color: {color}; border: 1.5px solid {color}; border-radius: 7px; padding: 4px 16px; background: #181818; letter-spacing: 1px;")
            value_label.setFixedHeight(32)
            value_label.setMinimumWidth(120)
            value_label.setAlignment(Qt.AlignVCenter | Qt.AlignLeft)

            if "Bağlantı" in title:
                self.status_labels['connection'] = value_label
            elif "Sıcaklık" in title:
                self.status_labels['temperature'] = value_label
            elif "Güç" in title:
                self.status_labels['power'] = value_label
            elif "Araç" in title:
                self.status_labels['vehicle'] = value_label

            row.addSpacing(8)
            row.addWidget(label)
            row.addSpacing(8)
            row.addWidget(value_label)
            row.addSpacing(8)
            right_layout.addLayout(row)
            right_layout.addSpacing(4)
        right_layout.addStretch()
        right_panel.setLayout(right_layout)

        center_panel = QFrame()
        center_panel.setStyleSheet("border: 2px solid #00ffe7; border-radius: 18px; background: #23272b;")
        self.panel_shadow(center_panel, '#00ffe7')
        center_layout2 = QVBoxLayout()
        self.webview = QWebEngineView()
        self.webview.setUrl(QUrl("http://192.168.1.100/"))
        self.webview.setMinimumSize(720, 480)
        center_layout2.addWidget(self.webview)
        center_panel.setLayout(center_layout2)

        center_layout.addWidget(left_panel)
        center_layout.addWidget(center_panel, stretch=2)
        center_layout.addWidget(right_panel)
        main_layout.addLayout(center_layout, stretch=2)
        main_layout.addWidget(bottom_panel)
        main_widget.setLayout(main_layout)
        self.setCentralWidget(main_widget)

    def update_sensor_data(self, data):
        try:
            print(f"Alınan ham veri: {data}")
            for key in data:
                try:
                    if key in ['ir_state', 'ir_state2']:
                        ir_value = str(data[key]).strip()
                        if ir_value == "":
                            self.sensor_data[key] = 0
                        else:
                            self.sensor_data[key] = int(float(ir_value))
                        print(f"IR Sensör {key} değeri: {self.sensor_data[key]}")
                    else:
                        # Diğer değerleri float'a çevir ve uygun şekilde işle
                        if key in ['total_distance', 'encoder_speed']:
                            try:
                                if key == 'total_distance':
                                    # Total distance milimetreden metreye çevir (100000'e böl)
                                    raw_value = float(data[key])
                                    meters = raw_value / 100000.0
                                    print(f"Total Distance dönüşüm: {raw_value}mm -> {meters}m")  # Debug
                                    self.sensor_data[key] = meters
                                    # Geriye dönük uyumluluk için encoder_pos'u da güncelle
                                    self.sensor_data['encoder_pos'] = meters
                                else:
                                    # Encoder hızını milimetreden metreye çevir
                                    self.sensor_data[key] = float(data[key]) / 1000.0
                            except (ValueError, TypeError) as e:
                                print(f"{key} dönüşüm hatası: {e}")
                                self.sensor_data[key] = 0.0
                        else:
                            # Diğer verileri normal şekilde işle
                            self.sensor_data[key] = float(data[key]) / 100.0
                except (ValueError, TypeError) as e:
                    print(f"Değer dönüşüm hatası - {key}: {data[key]}, Hata: {e}")
                    self.sensor_data[key] = 0.0

            print("Sensör verileri başarıyla işlendi:")
            for key, value in self.sensor_data.items():
                if key in ['ir_state', 'ir_state2']:
                    print(f"{key}: {int(value)}")
                elif key in ['total_distance']:
                    print(f"{key}: {value:.3f} m")
                elif key in ['encoder_speed']:
                    print(f"{key}: {value:.3f} m/s")
                else:
                    print(f"{key}: {value:.2f}")

            self.update_ui_labels()

            if hasattr(self, 'tunnel_bar') and self.tunnel_bar is not None:
                ir1_count = int(self.sensor_data.get('ir_state', 0))
                ir2_count = int(self.sensor_data.get('ir_state2', 0))
                common_count = min(ir1_count, ir2_count)
                # common_count'ı doğrudan pozisyon olarak kullan, max 100 olmalı
                calculated_distance = max(0, min(100, common_count))
                self.tunnel_bar.set_position(calculated_distance)



        except Exception as e:
            print(f"Veri işleme hatası: {e}")
            import traceback
            traceback.print_exc()

    def send_command(self, command):
        try:
            command_data = {"command": command}
            json_data = json.dumps(command_data)
            self.udp_sender.sendto(json_data.encode(), self.stm32_address)
            print(f"Komut gönderildi: {command}")
        except Exception as e:
            print(f"Komut gönderme hatası: {e}")

    def update_ui_labels(self):
        try:
            print("UI güncelleme başladı")
            if hasattr(self, 'acc_label') and self.acc_label:
                acc_text = f"X: {self.sensor_data['accel_x']:.2f} m/s²\nY: {self.sensor_data['accel_y']:.2f} m/s²\nZ: {self.sensor_data['accel_z']:.2f} m/s²"
                self.acc_label.setText(acc_text)

            if hasattr(self, 'orient_label') and self.orient_label:
                orient_text = f"X: {self.sensor_data['gyro_x']:.2f}°\nY: {self.sensor_data['gyro_y']:.2f}°\nZ: {self.sensor_data['gyro_z']:.2f}°"
                self.orient_label.setText(orient_text)

            if hasattr(self, 'pos_label') and self.pos_label:
                pos_text = f"Pos: {self.sensor_data.get('total_distance', 0.0):.2f} m"
                self.pos_label.setText(pos_text)

            if hasattr(self, 'speed_label') and self.speed_label:
                speed_text = f"Hız: {self.sensor_data.get('encoder_speed', 0.0):.2f} m/s"
                self.speed_label.setText(speed_text)

            if hasattr(self, 'temp_label') and self.temp_label:
                ambient_temp = self.sensor_data['temperature']
                object_temp = self.sensor_data['object_temp']
                temp_text = f"Kapsül: {ambient_temp:.1f}°C\nNesne: {object_temp:.1f}°C"
                self.temp_label.setText(temp_text)
                print(f"Sıcaklık güncellendi: {temp_text}")
                if ambient_temp > 60 or object_temp > 60:
                    temp_color = "#ff0000"
                elif ambient_temp > 45 or object_temp > 45:
                    temp_color = "#ffe700"
                else:
                    temp_color = "#00ffe7"
                self.temp_label.setStyleSheet(f"color: {temp_color}; padding: 2px 0px;")

            if hasattr(self, 'power_label') and self.power_label:
                current_ma = self.sensor_data.get('current', 0.0)  # mA cinsinden
                power_text = f"Akım: {current_ma:.1f} mA"
                self.power_label.setText(power_text)

            self.update_status_labels()

        except Exception as e:
            print(f"UI güncelleme hatası: {e}")
            import traceback
            traceback.print_exc()

    def update_status_labels(self):
        try:
            if 'connection' in self.status_labels:
                status = self.sensor_data['connection_status']
                temperature = self.sensor_data.get('temperature', 0)
                if status == "Başarılı" or status == "Connected":
                    if 0 <= temperature <= 100:
                        color = "#00ff00"
                        text = "Başarılı"
                    else:
                        color = "#ffe700"
                        text = "Veri Hatası"
                else:
                    color = "#ff0000"
                    text = "Bağlantı Bekleniyor"
                self.status_labels['connection'].setText(text)
                self.status_labels['connection'].setStyleSheet(
                    f"color: {color}; border: 1.5px solid {color}; border-radius: 7px; padding: 4px 16px; background: #181818; letter-spacing: 1px;"
                )

            if 'temperature' in self.status_labels:
                max_temp = max(self.sensor_data['temperature'], self.sensor_data['object_temp'])
                if max_temp > 60:
                    color = "#ff0000"
                    text = "Tehlikeli"
                elif max_temp > 45:
                    color = "#ffe700"
                    text = "Uyarı"
                else:
                    color = "#00ff00"
                    text = "Normal"
                self.status_labels['temperature'].setText(text)
                self.status_labels['temperature'].setStyleSheet(
                    f"color: {color}; border: 1.5px solid {color}; border-radius: 7px; padding: 4px 16px; background: #181818; letter-spacing: 1px;"
                )

            if 'power' in self.status_labels:
                power = abs(self.sensor_data['accel_x']) * 10 + abs(self.sensor_data['encoder_speed']) * 5
                if power > 100:
                    color = "#ff0000"
                    text = "Yüksek"
                elif power > 50:
                    color = "#ffe700"
                    text = "Orta"
                else:
                    color = "#00ff00"
                    text = "Normal"
                self.status_labels['power'].setText(text)
                self.status_labels['power'].setStyleSheet(
                    f"color: {color}; border: 1.5px solid {color}; border-radius: 7px; padding: 4px 16px; background: #181818; letter-spacing: 1px;"
                )

            if 'vehicle' in self.status_labels:
                speed = abs(self.sensor_data['encoder_speed'])
                if speed > 1:
                    color = "#00ff00"
                    text = "Hareket Halinde"
                else:
                    color = "#00ffe7"
                    text = "Durgun"
                self.status_labels['vehicle'].setText(text)
                self.status_labels['vehicle'].setStyleSheet(
                    f"color: {color}; border: 1.5px solid {color}; border-radius: 7px; padding: 4px 16px; background: #181818; letter-spacing: 1px;"
                )

        except Exception as e:
            print(f"Status güncelleme hatası: {e}")

    def update_connection_status(self, status):
        self.sensor_data['connection_status'] = status
        print(f"Bağlantı durumu güncellendi: {status}")
        if 'connection' in self.status_labels:
            if status == "Başarılı":
                color = "#00ff00"
                text = "Başarılı"
            else:
                color = "#ff0000"
                text = status
            self.status_labels['connection'].setText(text)
            self.status_labels['connection'].setStyleSheet(
                f"color: {color}; border: 1.5px solid {color}; border-radius: 7px; padding: 4px 16px; background: #181818; letter-spacing: 1px;"
            )

if __name__ == "__main__":
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    sys.exit(app.exec_())
