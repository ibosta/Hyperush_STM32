# 🚀 HYPERUSH 2025 - STM32 Multi-Sensor System

## 📋 Proje Açıklaması

Bu proje, **HYPERUSH 2025 Hyperloop Takımı**'nın resmi takım geliştirme aracıdır. STM32F756xx mikrodenetleyici kullanarak çoklu sensör entegrasyonu ve gerçek zamanlı veri toplama sistemi sağlar.

## 🛠️ Donanım Özellikleri

### 🔧 Ana Mikrodenetleyici

- **STM32F756xx** (ARM Cortex-M7, 216MHz)
- **200MHz** sistem saati
- **Dual I2C** bus desteği
- **Encoder** interface (TIM2)
- **UART** debug output


### 📡 Entegre Sensörler

| Sensör | Interface | Pinler | Açıklama |
| :-- | :-- | :-- | :-- |
| 🌡️ **MLX90614** | I2C1 | PB8 (SDA), PB9 (SCL) | Temassız sıcaklık sensörü |
| 🔄 **BNO055** | I2C2 | PF0 (SDA), PF1 (SCL) | 9-DOF IMU (İvmeölçer) |
| ⚡ **INA226** | I2C2 | SDA/SCL | Hassas akım/voltaj sensörü |
| 📏 **Rotary Encoder** | TIM2 | CH1 (A), CH2 (B) | 3600 PPR, 6mm tekerlek |
| 👁️ **IR Sensör** | GPIO | PF5 | Nesne algılama |

## 🎯 Özellikler

### ⚡ Gerçek Zamanlı Veri İzleme

- **Live Expressions** desteği (STM32CubeIDE)
- **UART Debug** output (115200 baud)
- **LED** status göstergeleri


### 📊 Ölçüm Yetenekleri

- 🌡️ **Sıcaklık**: Ortam ve nesne sıcaklığı (°C)
- ⚡ **Elektriksel**: Voltaj (V), akım (mA), güç (mW)
- 🔄 **Hareket**: İvme (3-axis), açısal konum
- 📏 **Mesafe**: Hassas mesafe ölçümü (mm/cm/m)
- 🎯 **Nesne Sayımı**: IR sensör ile


## 🚀 Kurulum ve Kullanım

### 📋 Gereksinimler

- STM32CubeIDE 1.8+
- STM32F756xx Discovery/Nucleo board
- HAL Driver Library


### 🔧 Kurulum Adımları

1. **Repository'yi klonlayın**
```bash
git clone https://github.com/ibosta/Hyperush_STM32.git
cd Hyperush_STM32
```

2. **STM32CubeIDE'de proje oluşturun**
    - New → STM32 Project
    - STM32F756xx board seçin
    - main.c dosyasını bu repository'den kopyalayın

3. **Build ve Upload**
    - Ctrl + B ile build edin
    - F11 ile debug modunda çalıştırın

## 📡 Pin Konfigürasyonu

### 🔌 I2C Bağlantıları

#### I2C1 (MLX90614 Sıcaklık Sensörü)

```
PB8  → SDA
PB9  → SCL
3.3V → VCC
GND  → GND
```


#### I2C2 (BNO055 + INA226)

```
PF0  → SDA
PF1  → SCL
3.3V → VCC
GND  → GND
```


### ⚙️ Encoder Bağlantısı

```
TIM2_CH1 → A sinyali
TIM2_CH2 → B sinyali
6mm tekerlek çapı
3600 PPR (pulse per revolution)
```


### 👁️ IR Sensör

```
PF5 → Digital Input
```


## 📊 Live Expressions Kullanımı

STM32CubeIDE'de debug modunda **Live Expressions** penceresini açın ve aşağıdaki değişkenleri izleyin:

### 🌡️ Sıcaklık Değişkenleri

```c
ambient_temp_C    // Ortam sıcaklığı (°C)
object_temp_C     // Nesne sıcaklığı (°C)
```


### ⚡ Elektriksel Değişkenler

```c
ina226_bus_voltage     // Bus voltajı (V)
ina226_current_ma      // Akım (mA)
ina226_power_mw        // Güç (mW)
ina226_connection_status // Bağlantı durumu (1/0)
```


### 🔄 Hareket Değişkenleri

```c
accel_x, accel_y, accel_z    // İvme değerleri
encoder_distance_mm          // Mesafe (mm)
encoder_distance_cm          // Mesafe (cm)
encoder_rotations           // Devir sayısı
encoder_speed_rpm           // Hız (RPM)
encoder_direction           // Yön (1=ileri, -1=geri, 0=durgun)
```


### 👁️ Algılama Değişkenleri

```c
ir_sensor_state      // IR sensör durumu (1/0)
ir_object_count      // Algılanan nesne sayısı
```


## 🎛️ Konfigürasyon

### 📏 Encoder Ayarları

```c
#define ENCODER_PPR 3600           // Pulse per revolution
#define WHEEL_DIAMETER_MM 6.0f     // Tekerlek çapı (mm)
```


### 📡 Sensör Adresleri

```c
#define MLX90614_ADDR (0x5A << 1)  // MLX90614 adresi
#define BNO055_ADDRESS (0x28 << 1) // BNO055 adresi
#define INA226_ADDR (0x40 << 1)    // INA226 adresi
```


## 🔍 Debug ve Test

### 🖥️ UART Terminal Ayarları

- **Baud Rate**: 115200
- **Data Bits**: 8
- **Stop Bits**: 1
- **Parity**: None


### 📋 Test Prosedürü

1. **Sistem başlatma kontrolü**
    - UART terminalinden başlatma mesajlarını kontrol edin
    - LED'lerin yanıp yanmadığını kontrol edin
2. **Sensör bağlantı testleri**
    - Her sensör için "bulundu" mesajını bekleyin
    - Device ID değerlerini kontrol edin
3. **Live Expressions kurulumu**
    - Debug modunda çalıştırın
    - Live Expressions penceresini açın
    - Yukarıdaki değişkenleri tek tek ekleyin
4. **Veri doğrulama**
    - Sensör değerlerinin mantıklı aralıklarda olduğunu kontrol edin
    - Encoder'ı manuel döndürüp değerlerin değiştiğini gözlemleyin

## 📈 Performans Karakteristikleri

### ⏱️ Okuma Hızları

- **Ana döngü**: 1000ms (1Hz)
- **Encoder güncelleme**: Gerçek zamanlı
- **Hız hesaplama**: 500ms aralıklarla
- **UART output**: 115200 baud


### 🎯 Hassasiyet Değerleri

- **Mesafe ölçümü**: ~0.00524mm per pulse
- **Sıcaklık ölçümü**: ±0.5°C hassasiyet
- **Voltaj ölçümü**: 1.25mV resolution
- **Akım ölçümü**: Shunt resistor değerine bağlı


## 📚 Eğitici Kaynaklar

### 📖 Datasheet'ler

- [STM32F756xx Reference Manual](https://www.st.com/resource/en/reference_manual/rm0385-stm32f75xxx-and-stm32f74xxx-advanced-armbased-32bit-mcus-stmicroelectronics.pdf)
- [MLX90614 Datasheet](https://www.melexis.com/en/documents/documentation/datasheets/datasheet-mlx90614)
- [BNO055 Datasheet](https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bno055-ds000.pdf)
- [INA226 Datasheet](https://www.ti.com/lit/ds/symlink/ina226.pdf)


### 🎓 Öğrenme Kaynakları

- [STM32CubeIDE Kullanım Kılavuzu](https://www.st.com/resource/en/user_manual/um2609-stm32cubeide-user-guide-stmicroelectronics.pdf)
- [I2C Protocol Tutorial](https://www.circuitbasics.com/basics-of-the-i2c-communication-protocol/)
- [Live Expressions Tutorial](https://www.st.com/resource/en/application_note/an5361-getting-started-with-projects-based-on-dualcore-stm32h7-microcontrollers-in-stm32cubeide-stmicroelectronics.pdf)


## 🔧 Geliştirme İpuçları

### 💡 Kod Optimizasyonu

- Volatile anahtar kelimesini live expressions değişkenleri için kullanın
- I2C timeout değerlerini sensör özelliklerine göre ayarlayın
- Encoder overflow kontrollerini mutlaka ekleyin


### 🐛 Debug İpuçları

- UART çıkışlarını her zaman kontrol edin
- Error handling fonksiyonlarını kullanın
- Live Expressions ile gerçek zamanlı değişkenleri izleyin


### ⚡ Performans İyileştirme

- Ana döngü süresini sensör gereksinimlerine göre ayarlayın
- Gereksiz HAL_Delay() çağrılarını azaltın
- DMA kullanarak I2C performansını artırın


## 🤝 Katkıda Bulunma

1. **Fork** edin
2. **Feature branch** oluşturun (`git checkout -b feature/amazing-feature`)
3. **Değişikliklerinizi commit** edin (`git commit -m 'Add amazing feature'`)
4. **Branch'inizi push** edin (`git push origin feature/amazing-feature`)
5. **Pull Request** açın

## 📄 Lisans

Bu proje **HYPERUSH Hyperloop Takımı** tarafından geliştirilmiştir.

## 👥 Takım

**HYPERUSH 2025 Hyperloop Takımı**

- 🚀 Hyper Loop Team
- 🌐 Website: [http://podhyperush.com/](http://podhyperush.com/)

<div align="center">

### 🏆 Made with ❤️ by HYPERUSH

**Design By ibos**

</div>
