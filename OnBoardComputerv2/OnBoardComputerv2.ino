#include "Rakieta.h"

/*
  void loop_flight()
  {
    Serial.println("=== TRYB FLIGHT: oczekuje na implementacje ===");
    delay(10000);
  }

  void loop_dump()
  {
    Serial.println("\n=== TRYB DUMP: odczyt danych z Flash ===");
    
    // Inicjalizacja SPI1 (powinna być już wykonana, ale upewniamy się)
    spi1.begin();
    if (flash.begin())
    {
      uint32_t jedec = flash.getJEDECID();
      Serial.print("[FLASH] OK, JEDEC: 0x");
      Serial.println(jedec, HEX);
    }
    else
    {
      Serial.println("[FLASH] BRAK ODPOWIEDZI");
    }
    Serial.println("Odczyt pierwszych 512 bajtow (hex + ascii):\n");
    
    const uint32_t startAddr = 0;
    const uint32_t length = 512;
    uint8_t buffer[16];
    char ascii[17];
    
    for (uint32_t addr = startAddr; addr < startAddr + length; addr += 16)
    {
      flash.readBuffer(addr, buffer, 16);
      
      // Wydruk hex
      Serial.printf("%08X: ", addr);
      for (int i = 0; i < 16; i++)
      {
        Serial.printf("%02X ", buffer[i]);
        ascii[i] = (buffer[i] >= 32 && buffer[i] <= 126) ? buffer[i] : '.';
      }
      ascii[16] = '\0';
      Serial.printf(" | %s\n", ascii);
    }
    
    Serial.println("\n--- Koniec dump. Przechodze do trybu SLEEP ---");
    delay(1000);
    loop_sleep(); // przejście do snu
  }
*/


Rakieta rakieta;

void setup()
{
  Serial.begin(115200);
  while (!Serial) delay(10);

  rakieta.init();
}

void loop()
{
  rakieta.loop();
}











