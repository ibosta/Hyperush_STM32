import socket
import time

# Yerel port ayarla (yanıtları almak için)
LOCAL_PORT = 8080  # Ana bilgisayarın dinleyeceği port socket
import time
StopAsyncIteration
# UDP istemsa
# ci ayarları
UDP_IP = "10.42.1.100"  # STM32'nin IP adresi
UDP_PORT = 8080  # STM32'nin dinlediği port

# UDP soketini oluştur
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.bind(('0.0.0.0', LOCAL_PORT))  # Yerel portu dinle
sock.settimeout(1.0)  # 1 saniye timeout

print(f"UDP Client başlatıldı. STM32 IP: {UDP_IP}, Port: {UDP_PORT}")

# Gönderilecek sayılar
numbers = [1, 2, 3, 4, 5, 6, 7, 8]

def send_message(message):
    try:
        # Mesajı gönder
        sock.sendto(message.encode('utf-8'), (UDP_IP, UDP_PORT))
        
        # Yanıtı bekle
        try:
            data, addr = sock.recvfrom(1024)
            print(f"STM32'den gelen yanıt: {data.decode('utf-8')}")
        except socket.timeout:
            print(f"Uyarı: Yanıt alınamadı")
            
    except Exception as e:
        print(f"Hata: {e}")

def main():
    print("UDP Client çalışıyor...")
    print("Mesaj göndermek için yazın (çıkmak için 'q' yazın):")
    
    while True:
        message = input("Gönderilecek mesaj: ")
        
        if message.lower() == 'q':
            break
            
        send_message(message)
        time.sleep(0.1)  # Kısa bir bekleme
    
    sock.close()
    print("\nUDP Client kapatıldı.")

if __name__ == "__main__":
    main()
