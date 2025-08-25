/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "lwip/pbuf.h"
#include "lwip/udp.h"
#include "lwip/tcp.h"
#include <string.h>
#include <stdio.h>
#include "stm32f7xx_hal.h"
#include "stm32f7xx_hal_gpio.h"
#include "udp_echoserver.h"

/* Private typedef -----------------------------------------------------------*/
volatile char received_string[MAX_STRING_LENGTH] = {0};  // Live expression'da görünecek string değişken
uint8_t current_speed = 0;  // Global motor hızı değişkeni

// Levitasyon kontrol değişkenleri
volatile uint8_t levitation_stop_request = 0;  // Durdurma talebi var mı?
volatile uint8_t levitation_active = 0;        // Levitasyon aktif mi?

void __udp_echoserver_receive_callback_oo(void *arg, struct udp_pcb *upcb, struct pbuf *p, const ip_addr_t *addr, u16_t port);
void CheckLevitationStop(void);  // Levitasyon durdurmayı kontrol et

void __udp_echoserver_init_oo(void)
{
	struct udp_pcb *upcb;

	/* Create a new UDP control block  */
	upcb = udp_new();

	if (upcb)
	{
		/* Bind the upcb to the UDP_PORT port */
		/* Using IP_ADDR_ANY allow the upcb to be used by any local interface */
		udp_bind(upcb, IP_ADDR_ANY, 8080);
		udp_recv(upcb, __udp_echoserver_receive_callback_oo, NULL);
	}
}

void __udp_echoserver_receive_callback_oo( void* arg,
                           struct udp_pcb* upcb,
                           struct pbuf* p,
                           const ip_addr_t* addr,
                           u16_t port )
{
    // Gelen string'i kopyala
    memset(received_string, 0, MAX_STRING_LENGTH);  // Önce buffer'ı temizle
    strncpy(received_string, (char *)p->payload, p->len < MAX_STRING_LENGTH ? p->len : MAX_STRING_LENGTH - 1);

    // Yanıt mesajı için buffer
    char response[150];

    // JSON komutunu ayrıştır
    if (strstr(received_string, "acil_durum")) {
        levitation_stop_request = 1;
        levitation_active = 0;        // Levitasyon artık aktif değil
        DecreaseMotorSpeed();         // Hızı sıfırla
                // Aktüvatörü durdur (PG5 = HIGH, PG6 = HIGH)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);   // PG5 HIGH
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);   // PG6 HIGH
        // Levitasyon durdur - durdurma talebi oluştur
        // Dur durumu: INPUT1=HIGH, INPUT2=HIGH
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_SET);    // PG3 HIGH
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);   // PD10 HIGH
    }
    else if (strstr(received_string, "sistem_baslat")) {

        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET); // PG5 LOW
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);   // PG6 HIGH
        HAL_Delay(6655);
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);   // PG5 HIGH
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);   // PG6 HIGH
    }
    else if (strstr(received_string, "ileri_al")) {
        // İleri: INPUT1=LOW, INPUT2=HIGH
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);  // PG3 LOW
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_SET);   // PD10 HIGH
        snprintf(response, sizeof(response), "Arac ileri gidiyor - PG3:LOW, PD10:HIGH");
    }
    else if (strstr(received_string, "geri_al")) {
        // Geri: INPUT1=LOW, INPUT2=LOW
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_3, GPIO_PIN_RESET);  // PG3 LOW
        HAL_GPIO_WritePin(GPIOD, GPIO_PIN_10, GPIO_PIN_RESET); // PD10 LOW
        snprintf(response, sizeof(response), "Arac geri gidiyor - PG3:LOW, PD10:LOW");
    }
    else if (strstr(received_string, "aktuator_yukari")) {
        // Aktüvatörü yukarı hareket ettir (PG5 = HIGH, PG6 = LOW)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);   // PG5 HIGH
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_RESET); // PG6 LOW
        snprintf(response, sizeof(response), "Aktüvatör yukarı gidiyor - PG5:HIGH, PG6:LOW");
    }
    else if (strstr(received_string, "aktuator_asagi")) {
        // Aktüvatörü aşağı hareket ettir (PG5 = LOW, PG6 = HIGH)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_RESET); // PG5 LOW
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);   // PG6 HIGH
        snprintf(response, sizeof(response), "Aktüvatör aşağı gidiyor - PG5:LOW, PG6:HIGH");
    }
    else if (strstr(received_string, "aktuator_durdur")) {
        // Aktüvatörü durdur (PG5 = HIGH, PG6 = HIGH)
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_5, GPIO_PIN_SET);   // PG5 HIGH
        HAL_GPIO_WritePin(GPIOG, GPIO_PIN_6, GPIO_PIN_SET);   // PG6 HIGH
        snprintf(response, sizeof(response), "Aktüvatör durdu - PG5:HIGH, PG6:HIGH");
    }
    else if (strstr(received_string, "levitasyon_baslat")) {
        // Levitasyon başlat - yeni yavaş hızlanma mantığı
        levitation_stop_request = 0;  // Durdurma talebini sıfırla
        levitation_active = 1;        // Levitasyon aktif

        snprintf(response, sizeof(response), "Levitasyon başlatıldı - Yavaş hızlanma modunda");
    }
    else if (strstr(received_string, "levitasyon_durdur")) {
        // Levitasyon durdur - durdurma talebi oluştur
        levitation_stop_request = 1;
        levitation_active = 0;        // Levitasyon artık aktif değil
        DecreaseMotorSpeed();         // Hızı sıfırla
        snprintf(response, sizeof(response), "Levitasyon durdurma talebi gönderildi - Motor hızı sıfırlandı");
    }
    else {
        snprintf(response, sizeof(response), "Bilinmeyen komut: %s", received_string);
    }

    // Yanıt için yeni pbuf oluştur
    struct pbuf* p_reply = pbuf_alloc(PBUF_TRANSPORT, strlen(response) + 1, PBUF_RAM);
    if (p_reply != NULL) {
        memcpy(p_reply->payload, response, strlen(response) + 1);

        // Yanıtı gönder
        udp_sendto(upcb, p_reply, addr, port);

        // Yanıt pbuf'ını serbest bırak
        pbuf_free(p_reply);
    }

    // Gelen veri pbuf'ını serbest bırak
    pbuf_free(p);
}

// Bu fonksiyonu interrupt veya başka bir thread'den çağırabilirsiniz
void CheckLevitationStop(void)
{
    // Eğer durdurma talebi varsa hızı sıfırla ve çık
    if (levitation_stop_request) {
        levitation_active = 0;  // Levitasyon artık aktif değil
        DecreaseMotorSpeed();
    }
}

// Yeni fonksiyon: Levitasyon sürecini yönet (main loop'ta çağrılacak)
void ProcessLevitation(void)
{
    static uint32_t last_speed_update = 0;
    uint32_t current_time = HAL_GetTick();




    // Levitasyon aktifse ve durdurma talebi yoksa
    if (levitation_active && !levitation_stop_request) {
        // Her 200ms'de bir hız artışı (eskiden 50ms idi - 4 kat daha yavaş)
        if (current_time - last_speed_update >= 200) {
            if (current_speed < 100) {
                IncreaseMotorSpeed();  // Hızı artır
                last_speed_update = current_time;
            } else {
                // %100'e ulaştık, levitasyon tamamlandı
                levitation_active = 0;
            }
        }
    }

    // Her durumda durdurma talebini kontrol et
    CheckLevitationStop();
}
