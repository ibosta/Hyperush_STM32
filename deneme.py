from PyQt5.QtWidgets import QWidget, QApplication
from PyQt5.QtGui import QPainter, QColor, QBrush, QPen
from PyQt5.QtCore import Qt, QRect
import sys

class HyperloopTunnel(QWidget):
    def __init__(self, parent=None, min_pos=0, max_pos=100):
        super().__init__(parent)
        self.min_pos = min_pos
        self.max_pos = max_pos
        self.position = min_pos
        self.setMinimumHeight(60)
        self.setMaximumHeight(70)
        self.setStyleSheet("background: transparent;")

    def set_position(self, pos):
        # Pozisyonu min-max aralığında sınırla
        self.position = max(self.min_pos, min(self.max_pos, pos))
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        rect = self.rect().adjusted(10, 20, -10, -20)
        painter.setRenderHint(QPainter.Antialiasing)

        # Tünel çizgisi
        tunnel_height = 20
        tunnel_rect = QRect(rect.left(), rect.center().y() - tunnel_height // 2, rect.width(), tunnel_height)
        painter.setPen(QPen(QColor('#00ffe7'), 3))
        painter.setBrush(QColor(30, 30, 30))
        painter.drawRoundedRect(tunnel_rect, 10, 10)

        # Araç (sadelestirilmiş, mavi dikdörtgen)
        total_range = self.max_pos - self.min_pos
        rel_pos = (self.position - self.min_pos) / total_range
        tunnel_length = tunnel_rect.width()
        car_width = 40
        car_height = 30
        car_x = int(tunnel_rect.left() + rel_pos * tunnel_length) - car_width // 2
        car_y = tunnel_rect.center().y() - car_height // 2

        car_rect = QRect(car_x, car_y, car_width, car_height)
        painter.setPen(QPen(QColor('#00aaff'), 2))
        painter.setBrush(QBrush(QColor('#00ccff')))
        painter.drawRoundedRect(car_rect, 8, 8)

        # Araç üstünde pencere veya yüzey simülasyonu
        window_rect = QRect(car_x + 10, car_y + 7, 20, 15)
        painter.setBrush(QColor('#80dfff'))
        painter.drawRoundedRect(window_rect, 5, 5)

        # Tünel başı ve sonu metinleri
        painter.setPen(QColor('#00ffe7'))
        painter.drawText(tunnel_rect.left(), tunnel_rect.bottom() + 15, "0m")
        painter.drawText(tunnel_rect.right() - 20, tunnel_rect.bottom() + 15, f"{self.max_pos}m")


if __name__ == "__main__":
    app = QApplication(sys.argv)

    window = QWidget()
    window.setWindowTitle("Hyperloop Tunnel Position Demo")
    window.setGeometry(200, 200, 800, 120)
    layout = QVBoxLayout(window)

    tunnel = HyperloopTunnel(max_pos=100)
    layout.addWidget(tunnel)

    from PyQt5.QtCore import QTimer
    pos = 0

    def update_position():
        nonlocal pos
        pos += 1
        if pos > 100:
            pos = 0
        tunnel.set_position(pos)

    timer = QTimer()
    timer.timeout.connect(update_position)
    timer.start(50)  # 20 FPS civarı hareket sağlar

    window.show()
    sys.exit(app.exec_())
