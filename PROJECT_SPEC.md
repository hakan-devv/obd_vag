# Proje: VAG OBD/UDS Araç Gösterge Cihazı
## ONX2432G028 (ESP32-S3 + ST7789) + MCP2515

## 1. Çalışma şekli ve genel kurallar

- Geliştirme ortamı: ESP-IDF v5.5.2, C/C++, `idf.py`, VS Code + Espressif eklentisi.
- Arduino framework KULLANILMAYACAK.
- Dosya oluşturma veya değiştirme gerektiğinde her dosyanın tam yolunu ve eksiksiz içeriğini ver. Parça kod veya `...` ile atlanmış içerik verme.
- Her seferinde yalnızca talep ettiğim fazı uygula. Sonraki faza kendiliğinden geçme.
- Hayali API, fonksiyon veya register adı uydurma. Kullanılan kütüphanenin gerçek kaynak kodunu incele.
- Emin olmadığın bir şeyi kesin bilgi gibi sunma; `DOĞRULANACAK` olarak işaretle.
- Donanım pinlerini kendi kafana göre değiştirme.
- Üçüncü taraf kaynak kodların lisans bildirimlerini ve copyright başlıklarını koru.
- Bir fazın tamamlandığını iddia etmeden önce ilgili derleme/test sonuçlarını göster. Fiziksel olarak test edilmemiş donanım davranışlarını doğrulanmış gibi anlatma.

## 2. Projenin amacı

ONX2432G028 kartındaki ESP32-S3 ile harici MCP2515 CAN kontrolcüsü üzerinden araçların diagnostik verilerini okumak ve bunları kartın dahili ST7789 LCD ekranında göstermek.

İlk hedefler:

1. MCP2515 ile güvenilir SPI haberleşmesi.
2. Doğrulanmış 500 kbps CAN bit timing.
3. Loopback self-test.
4. Güvenli şekilde fiziksel CAN hattını dinleme.
5. Standart OBD-II Mode 01 PID desteği.
6. ISO-TP Single Frame ve temel UDS servisleri.
7. VAG'a özel veriler için genişletilebilir ECU profili.
8. Canlı parametreleri ONX2432G028 kartının dahili ST7789 LCD ekranında göstermek.

Bu fazlarda temel öncelik görsel tasarım değil, doğru veri, güvenilir haberleşme ve test edilebilir bir mimaridir.

## 3. Donanım

### 3.1 Ana kart

- Kart: ONX2432G028
- MCU: ESP32-S3R8
- CPU: Xtensa LX7, 240 MHz
- Flash: 16 MB
- PSRAM: 8 MB
- Dahili ST7789 LCD: BU PROJEDE KULLANILACAK; çözünürlük 320×240.
- Dahili CST826 dokunmatik: bu temel RPM ekranı aşamasında kullanılmayacak.
- GPIO7/GPIO8, ekran resetini kontrol eden PCF8574 genişleticisi için I²C0 hatlarıdır; bu aşamada dokunmatik sürücüsü başlatılmayacak.

Kartın resmî açık kaynak donanım ve ESP-IDF örneklerini incele:

https://github.com/OpenNextion/OpenNextion-SKU-ONX2432G028

Flash boyutu, flash modu, flash frekansı ve PSRAM ayarlarını kartın gerçek donanımı ile resmî ESP-IDF örneğinden doğrula. Flash modunu tahmin etme.

Önerilen başlangıç yapılandırması:

- ESP-IDF target: `esp32s3`
- Flash size: 16 MB
- `CONFIG_FREERTOS_HZ=1000`
- PSRAM kullanılmayacak; gereksiz PSRAM başlatması ekleme.

### 3.2 MCP2515 CAN modülü

Fiziksel olarak doğrulanan modül bilgileri:

- CAN controller: MCP2515
- CAN transceiver: TJA1050
- Kristal: 8 MHz
- Transceiver/modül beslemesi: 5 V
- CAN hızı: 500 kbps
- CAN frame formatı: ilk profilde standard 11-bit identifier
- INT pini: KULLANILMAYACAK; polling yapılacak.

Bağlantılar, ONX2432G028'in 24-pin FPC expansion bağlantısından uygun breakout ile çıkarılacaktır.

| ONX2432G028 | MCP2515 modülü |
|---|---|
| GPIO38 | SCK |
| GPIO39 | MOSI / SI |
| GPIO40 | MISO / SO |
| GPIO42 | CS |
| GND | GND |
| Uygun 5 V besleme | VCC |

Besleme, lojik seviye dönüşümü, MISO koruması ve ortak toprak bağlantısı donanım tarafında çözülecektir. Firmware'e yazılımsal level-shifter ekleme.

MCP2515 SPI bağlantısı için:

- ESP-IDF `spi_master` kullanılacak.
- Host: `SPI2_HOST`.
- GPIO'lar belirtilen şekilde GPIO matrix üzerinden atanacak.
- Başlangıç SPI hızı: 1 MHz.
- SPI hızı konfigürasyondan değiştirilebilir olacak; üst sınır 5 MHz.
- SPI mode: sürücünün gerçek gereksinimi ve MCP2515 datasheet'i doğrulanarak belirlenecek.
- CS yönetimi ESP-IDF SPI driver üzerinden yapılacak.

Kullanılmaması gereken pinler:

- GPIO45: boot strapping pinidir; CS veya kontrol pini olarak kullanma.
- GPIO35/36/37: Octal PSRAM ile ilişkili pinler; kullanma.
- GPIO9/10/11 ve EXIO6: ileride MicroSD için ayrılmış; bu fazda kullanma.

### 3.3 Dahili ONX2432G028 LCD

- Panel: dahili ST7789, 320×240 RGB565.
- LCD SPI pinleri: SCLK GPIO5, MOSI GPIO1, DC GPIO3, CS GPIO2.
- LCD arka aydınlatması: GPIO6.
- LCD reseti: I²C0 üzerinden PCF8574 (7-bit adres 0x38), GPIO7 SCL / GPIO8 SDA; reset çıkışı expander P5.
- MCP2515 SPI2_HOST kullandığı için LCD ayrı SPI3_HOST üzerinde çalıştırılacak.
- Bu panel, UART1'e Nextion komutları gönderilen harici HMI değildir. LCD, ST7789 panel sürücüsü ile doğrudan başlatılır.

## 4. MCP2515 sürücüsünün entegrasyonu

Kullanılacak repository:

https://github.com/Microver-Electronics/mcp2515-esp32-idf

Bu repository, autowp MCP2515 kütüphanesinin ESP-IDF için C portudur.

Entegrasyondan önce:

1. Repository'nin gerçek kaynak kodunu incele.
2. Kullanılan commit hash'ini kaydet.
3. Dosya uzantılarını, lisansı, public API'leri ve CMake gereksinimlerini doğrula.
4. README'deki kod örneklerini kaynak kodla karşılaştır. Doğrulanmamış API'yi kullanma.
5. `components/mcp2515/` altında component olarak entegre et.
6. Gereken kaynak/header dosyalarını ve lisans dosyasını koru.
7. ESP-IDF v5.5.2 ile uyumlu bir `CMakeLists.txt` hazırla.
8. Gerekli ESP-IDF component bağımlılıklarını gerçek kullanım üzerinden belirle.
9. Kütüphanenin init sırasını incele; SPI bus başlatma, SPI device ekleme, MCP2515 reset ve register yazma sırasını varsayma.

Sürücünün mevcut API'sinde register okuma/yazma, loopback, Listen-Only, normal mode, CAN gönderme/alma ve maske/filtre fonksiyonları olup olmadığını doğrula.

Gerekirse sürücü kodunda sınırlı değişiklik yap; değişiklikleri ayrı ve anlaşılır tut.

## 5. MCP2515 bit timing — kritik gereksinim

Amaç: 8 MHz kristal ile 500 kbps CAN.

MCP2515 datasheet'i:

https://ww1.microchip.com/downloads/en/DeviceDoc/MCP2515-Family-Data-Sheet-DS20001801K.pdf

Bu kombinasyon için hesap:

- `TQ = 2 × (BRP + 1) / Fosc`
- BRP = 0 olduğunda TQ = 250 ns.
- Bit süresi = 2 µs.
- Bit başına toplam 8 TQ.
- PS2 en az 2 TQ olduğundan sample point en fazla %75 olabilir.
- `%87.5` sample point bu kombinasyonda mümkün değildir; böyle bir hedef varsayma.

Datasheet'e göre doğrulanacak aday register değerleri:

| Register | Aday değer | Anlamı |
|---|---:|---|
| CNF1 | `0x00` | BRP=0, SJW=1 TQ |
| CNF2 | `0x91` | BTLMODE=1, PS1=3 TQ, PropSeg=2 TQ |
| CNF3 | `0x01` | PS2=2 TQ |

Aday konfigürasyonda:

`SyncSeg 1 + PropSeg 2 + PS1 3 + PS2 2 = 8 TQ`

Sample point:

`(1 + 2 + 3) / 8 = %75`

Bu değerler kodlanmadan önce datasheet register bit düzenine göre doğrulanmalıdır. Hesabı ve her register alanının bit karşılığını raporla.

Repository'nin hazır bitrate tablosundaki 8 MHz / 500 kbps preset'ini de gerçek kaynak koddan bul ve hesapla. `0x00 / 0x90 / 0x82` olduğu bekleniyor; fakat bu değerleri varsayılan doğru kabul etme, fiziksel/kaynak doğrulaması yap.

Bu beklenen preset, hesaplandığında `SyncSeg 1 + PropSeg 1 + PS1 3 + PS2 3 = 8 TQ` ve %62,5 sample point sonucunu verir. Bunun datasheet hesabını kendin doğrula.

Hazır preset hedefi karşılamıyorsa bunu açıkça raporla ve component içinde datasheet'e uygun özel bit timing konfigürasyonu uygula. Üçüncü taraf kaynak değişikliklerinde lisans bildirimlerini koru.

Bit timing register'larını konfigürasyon modunda yazdıktan sonra geri oku:

- CNF1
- CNF2
- CNF3

Yazılan ve okunan değerleri karşılaştır. Başarı veya hata bilgisini logla.

CAN bit timing hesaplarını ve yazılan register değerlerini raporda ayrı ayrı göster. Hesaplanmış bitrate ile gerçek fiziksel bus haberleşmesini aynı doğrulama olarak sunma.

## 6. Yazılım mimarisi

Katmanların bağımlılık yönü yukarıdan aşağıya olacak:

```text
Nextion UI
    ↑
Parameter Manager
    ↑
OBD-II Mode 01        UDS
         \            /
           ISO-TP SF
               ↑
           CAN Driver
               ↑
             MCP2515
               ↑
          CAN Transceiver
               ↑
             Araç CAN
```

OBD-II Mode 01 ve UDS ayrı application-layer protokolleridir. ISO-TP ortak taşıma katmanı olabilir, ancak OBD-II UDS'nin alt kümesi olarak modellenmemelidir.

Üst katmanlar MCP2515'in API'sine doğrudan erişmemelidir.

### 6.1 CAN driver abstraction

`components/can_driver/` altında donanımdan bağımsız bir C arayüzü oluştur.

C uyumlu header taslağı:

```c
#ifndef CAN_DRIVER_H
#define CAN_DRIVER_H

#include <stdbool.h>
#include <stdint.h>
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

typedef struct {
    uint32_t id;
    uint8_t dlc;
    uint8_t data[8];
    bool extended;
} can_frame_t;

esp_err_t can_init(void);
esp_err_t can_send(const can_frame_t *frame);
esp_err_t can_receive(can_frame_t *frame);
esp_err_t can_available(bool *available);

#ifdef __cplusplus
}
#endif

#endif
```

Bu arayüzü gerçek sürücünün davranışına göre doğrula ve gerekli düzeltmeleri gerekçelendir.

Fonksiyon sözleşmesi:

- `can_init()`: driver başlatma.
- `can_send()`: geçerli frame gönderme.
- `can_receive()`: bloklamayan alım; frame yoksa `ESP_ERR_NOT_FOUND`, başarıda `ESP_OK`.
- `can_available(&available)`: `ESP_OK` döndürür ve `available` alanına RX frame durumunu yazar; SPI/durum hatasını ayrıca döndürür.

Geçersiz DLC, geçersiz pointer ve desteklenmeyen frame tipi gibi hataları ele al.

MCP2515'e özgü kod sadece CAN driver katmanında bulunmalı. İleride native ESP32-S3 TWAI'ye geçiş yapılabilmeli.

### 6.2 ISO-TP katmanı

Bu fazlar için yalnızca Single Frame (SF) implement edilecek.

Classic CAN normal addressing formatı:

- Byte 0 üst nibble: `0x0`.
- Byte 0 alt nibble: uygulama payload uzunluğu N.
- `N <= 7`.
- Payload Byte 1'den başlar.
- CAN DLC=8 kullanılacak; kalan byte'lar yapılandırılabilir padding ile doldurulacak.
- Varsayılan padding: `0x00`.

First Frame, Consecutive Frame veya Flow Control isteyen mesajlar bu fazda desteklenmeyecek. Böyle bir frame alındığında yanlışlıkla SF olarak decode etme. `UNSUPPORTED_MULTIFRAME` gibi tanımlı bir hata döndür ve logla.

Taşıma katmanı üst katmana yalnızca tamamlanmış application payload sunmalı.

### 6.3 OBD-II Mode 01

OBD-II, UDS değildir; ayrı modül olarak implement edilecek.

İlk profil:

- Functional request ID: `0x7DF`.
- Motor ECU için örnek physical request ID: `0x7E0`.
- Motor ECU için örnek physical response ID: `0x7E8`.
- Response filtresi: `0x7E8–0x7EF`.

Adresler evrensel değildir; vehicle profile tarafından yönetilmelidir.

RPM örneği:

Request CAN ID `0x7DF`:

```text
02 01 0C 00 00 00 00 00
```

Örnek response CAN ID `0x7E8`:

```text
04 41 0C A B 00 00 00
```

Hesap:

`RPM = ((A × 256) + B) / 4`

Decoder yalnızca gerekli veri uzunluğu ve doğru service/PID başlıkları doğrulandıktan sonra çalışmalı.

#### Supported PIDs

Supported PID capability discovery desteklenecek.

Örnek sorgular:

- `01 00`: PID 01–20 bitmap'i.
- `01 20`: PID 21–40 bitmap'i.
- `01 40`: PID 41–60 bitmap'i.
- Sonraki bitmap grupları yalnızca önceki bitmap'in devam biti destek gösteriyorsa istenecek.

Bitmap çözümlemesini ilgili OBD/J1979 standardına göre doğrula. Desteklenen PID'leri `SupportedPidSet` veya benzeri bir yapıda cache'le. Her PID sorgusundan önce capability discovery yapma.

Desteklenmediği bilinen PID'ler tekrar tekrar sorgulanmamalı. UI'da `--` gösterilmeli.

Bir PID'nin standartta tanımlı olması, ECU tarafından desteklendiği anlamına gelmez.

### 6.4 Standart OBD-II parametre adayları

Aşağıdaki parametreleri, desteklenme durumuna ve doğru decoder/formül doğrulamasına bağlı olarak ekle:

- RPM — `0x0C`
- Vehicle Speed — `0x0D`
- Calculated Engine Load — `0x04`
- Throttle Position — `0x11`
- Accelerator Pedal Position D/E — `0x49 / 0x4A`
- MAF — `0x10`
- IAT — `0x0F`
- MAP — `0x0B`
- Ignition Timing Advance — `0x0E`
- Coolant Temperature — `0x05`
- STFT/LTFT — ilgili Bank 1/2 PID'leri
- O2 sensor voltage ve equivalence ratio — desteklenen ilgili PID'ler
- Fuel pressure / fuel rail pressure — ilgili standart PID'ler, uygun birim ve formül doğrulanarak
- Fuel Level — `0x2F`
- Control Module Voltage — `0x42`
- Catalyst Temperature — `0x3C–0x3F`
- Ambient Air Temperature — `0x46`
- Engine Oil Temperature — `0x5C`
- Actual Engine Percent Torque — `0x62`
- Engine Reference Torque — `0x63`

`0x62` doğrudan Nm cinsinden tork olarak gösterilmemeli; standart PID anlamı ve dönüşüm formülü doğrulanmalı. `0x63` referans tork değeridir.

Hedef lambda, hedef boost, silindir bazlı knock retard, N75 duty ve benzeri OEM-specific değerleri standart PID varmış gibi kodlama.

Her decoder için kullanılan kaynak, formül, veri uzunluğu ve birim açık olsun.

### 6.5 UDS katmanı

UDS, ISO-TP SF üzerinde ayrı bir modüldür.

İlk desteklenecek servisler:

- `0x10 0x03`: DiagnosticSessionControl, extended session.
- `0x3E 0x00`: TesterPresent.
- `0x22`: ReadDataByIdentifier.

Extended session başarılı sayılmadan önce pozitif cevap doğrulanmalı:

```text
50 03 ...
```

Negatif cevap formatı:

```text
7F <original SID> <NRC>
```

NRC, service ve request bilgilerini logla.

NRC `0x78` (ResponsePending) geldiğinde yanıt bekleme durumunu uygun timeout ile sürdür. Aynı isteği gereksiz yere tekrar tekrar gönderme. P2/P2* zamanlamalarını cevap içinde sağlanıyorsa işle; aksi durumda yapılandırılabilir timeout kullan.

TesterPresent yaklaşık 2 saniyede bir, yalnızca yönetilen UDS session aktifken gönderilecek. Response ve timeout davranışı açıkça yönetilecek.

Session timeout, ECU bağlantı kesilmesi ve iletişim hataları için state machine kullan. Pozitif session cevabı olmadan `UDS_ACTIVE` durumuna geçme.

### 6.6 Vehicle profile

Araç bağımlı parametreleri profile sistemiyle tanımla.

Profil şunları içermeli:

- manufacturer
- platform/model/engine gibi tanımlayıcı bilgiler
- ECU family ve software bilgisi mevcutsa
- physical request ID
- physical response ID
- functional request ID
- padding byte
- protokol ve desteklenen özellikler
- parametre listesi

İlk profil: VAG / MED17.5.5.

Bu profil, bütün VAG araçlarının aynı adresleri veya aynı DID değerlerini kullandığı anlamına gelmez. Otomatik araç tanıma bu fazda yok; profil manuel seçilebilir.

### 6.7 OEM-specific DID profili

VAG'a özgü parametreler profil bazlı tasarlanacak.

Örnek adaylar:

- Requested/Target Boost
- Knock Retard
- Engine Oil Pressure
- N75 Duty Cycle
- Target Lambda

DID'ler ECU ailesi, ECU software versiyonu ve diagnostik tanımına göre değişebilir.

DID uydurma. Doğrulanmamış her DID `UNKNOWN/TODO` olarak işaretlenmeli ve varsayılan olarak gönderilmemeli.

Security Access gerekiyorsa bu projede bypass etme. Yazma, kodlama, programming veya actuator-control servisleri ekleme.

### 6.8 Parameter Manager

Her parametre için ortak bir model kullan.

Alanlar:

- `name`
- `source`: `OBD_PID` veya `UDS_DID`
- `id`: PID veya DID
- `response_offset`
- `response_length`
- `byte_order`
- `signed`
- `scale`
- `offset`
- `formula` veya `custom_decoder`
- `unit`
- `min_value`
- `max_value`
- `update_period_ms`
- `availability`
- `validity state`

`response_offset` değerinin tam olarak hangi noktaya göre tanımlandığını dokümante et. Önerilen kural: ISO-TP başlığı, UDS/OBD service byte'ları ve PID/DID başlıkları ayıklandıktan sonraki parametre veri alanına göre offset kullanmak.

Validity durumları en azından şunları kapsamalı:

- `VALID`
- `UNAVAILABLE`
- `UNSUPPORTED`
- `TIMEOUT`
- `STALE`
- `INVALID`

Süresi dolan veri eskimiş haliyle süresiz gösterilmemeli. Desteklenmeyen parametre ile timeout nedeniyle okunamayan parametre birbirinden ayırt edilmeli.

Decoder'lar donanımdan bağımsız saf fonksiyonlar olarak yazılmalı; Unity testleriyle test edilebilir olmalı.

### 6.9 Dahili LCD UI

`components/onx_display/` ONX2432G028 üzerindeki ST7789 panelini başlatır ve temel RPM ekranını çizer. Başlangıçta RPM `----` görünür; yalnızca geçerli ECU RPM cevabı çözümlendiğinde sayısal değer gösterilir.

Ekran bileşeni CAN sürücüsüne doğrudan erişmez; OBD istemcisinden güncel RPM ve durum bilgilerini alır. Daha sonra LVGL ve CST826 dokunmatik desteği ayrı bir fazda eklenebilir.

## 7. CAN RX polling ve filtreler

INT pini kullanılmayacak.

RX polling non-blocking olmalı. Gereksiz busy-wait ve uzun `delay()` kullanma.

Daha yüksek katmanlar CAN driver üzerinden frame alacak. MCP2515 `READ STATUS`, `CANINTF` ve RX buffer durumlarıyla ilgili yöntemleri sürücünün gerçek API'sinden doğrula.

### Donanım filtresi

Normal çalışma için yalnızca standard 11-bit `0x7E8–0x7EF` response aralığını kabul etmeye yönelik MCP2515 mask/filter konfigürasyonu kullanılacak.

Aday konfigürasyon:

- RXM0 maskesi: `0x7F8`
- RXM1 maskesi: `0x7F8`
- RXF0–RXF5: `0x7E8`
- Frame tipi: standard 11-bit

Bu maskenin `0x7E8–0x7EF` aralığını kabul ettiğini ve diğer ID'leri reddettiğini sürücünün ID kodlamasını dikkate alarak doğrula.

İki maskenin ve altı filtrenin tamamını yapılandır. Hiçbir maskeyi veya filtreyi yanlışlıkla açık bırakma. Gerçek filtre API'sinin standard/extended ID parametrelerini doğru kullan.

### Loopback filtre istisnası

Loopback self-test sırasında yukarıdaki response filtresi devre dışı kalmalı veya test ID'siyle uyumlu olmalı.

MCP2515 RXBnCTRL.RXM bitlerini loopback sırasında tüm geçerli frame'leri alacak şekilde ayarlayabilirsin; ancak bunu datasheet ve sürücü API'sine göre doğrula.

Loopback test tamamlanınca normal işletim için RX maskeleri ve filtreleri yeniden yapılandır.

Filtreler nedeniyle test frame'inin düşmesi, CAN controller veya SPI arızası olarak yanlış yorumlanmamalı.

## 8. Çalışma zamanı ve görevler

Bu mimari ileriki fazlarda FreeRTOS task ve queue yapısıyla uygulanacak.

Öngörülen görevler:

- CAN RX polling
- Diagnostic scheduler
- LCD/UI

Task'lar kuyruklarla haberleşmeli; UI doğrudan CAN driver'a bağlanmamalı.

Blocking loop ve uzun `delay()` kullanma. Polling için uygun `vTaskDelay(1)`/scheduler yaklaşımını kullan; CPU'yu gereksiz yere tamamen meşgul bırakma.

Sistem durumları:

- `BOOT`
- `CAN_INIT`
- `CAN_ERROR`
- `OBD_READY`
- `UDS_STARTING`
- `UDS_ACTIVE`
- `UDS_ERROR`

Gelecek fazlarda timeout ve eski veri durumları da parametre seviyesinde yönetilecek.

## 9. Güvenlik ve araç üzerinde test

Bu proje salt okunur bir diagnostik cihazdır.

İzin verilen diagnostik işlemler:

- OBD-II Mode 01 PID read
- UDS `0x10` session control
- UDS `0x3E` TesterPresent
- UDS `0x22` ReadDataByIdentifier

Aşağıdakiler eklenmeyecek:

- `0x2E` WriteDataByIdentifier
- `0x2F` InputOutputControl
- `0x31` RoutineControl
- ECU coding/programming
- key/security access bypass
- bilinmeyen CAN ID'lerine rastgele frame gönderme
- rastgele ham CAN TX

### İlk fiziksel araç bağlantısı

Araca ilk bağlanışta MCP2515 **Listen-Only** modunda olmalı ve hiçbir frame gönderilmemeli.

Listen-Only modunda CAN hattını dinle ve alınan frame'leri logla. Trafik görünmemesi tek başına hata kanıtı değildir; gateway davranışı, ECU aktivitesi, bağlantı ve bitrate ayrı ayrı değerlendirilmelidir.

Önemli: Listen-Only modunda MCP2515'in acceptance filter davranışının normal modla aynı olduğunu varsayma. İlk dinleme aşamasında gelen frame'ler uygulama katmanında filtrelenebilir.

Listen-Only modunda hata sayaçlarının/flag'lerinin davranışını datasheet'e göre değerlendir. EFLG tek başına bitrate veya fiziksel bağlantı doğruluğunun kesin kanıtı olarak kullanılmamalı; `CANINTF.MERRF` ve RX overflow durumları da uygun şekilde izlenmeli.

Normal modda fiziksel CAN TX yalnızca izin verilen, tanımlanmış diagnostik isteklerle yapılmalı. Faz 0'da araç hattına fiziksel TX yapılmayacak.

## 10. Logging

ESP-IDF logging kullanılacak.

Kategoriler:

- ERROR
- WARN
- INFO
- DEBUG
- TRACE

ESP-IDF'nin gerçek log makrolarını kullan; TRACE gerekiyorsa uygun `ESP_LOGV` tabanlı wrapper oluştur.

İleriki fazlarda şu olaylar loglanabilmeli:

- CAN TX/RX
- init ve register değerleri
- OBD request/response
- UDS request/response
- NRC
- timeout
- unsupported message
- decoder sonucu
- validity değişimi
- RX buffer overflow ve hata durumları

Ham veri ve debug logları gerektiğinde kapatılabilir olmalı.

## 11. Faz planı

Fazlar sırayla ilerleyecek. Her fazın tamamlanması test sonuçlarına dayalı olmalı.

### Faz 0 — Proje iskeleti ve MCP2515 doğrulaması

- ESP-IDF v5.5.2 proje iskeleti.
- `components/` yapısı ve CMake dosyaları.
- MCP2515 component entegrasyonu.
- SPI bus/device init, başlangıç hızı 1 MHz.
- MCP2515 reset ve register okuma/yazma testi.
- Repository commit hash'ini kaydet.
- Gerçek kaynak koddan CNF bitrate preset'ini bul.
- Datasheet'e göre bit timing hesabını göster.
- Aday CNF register değerlerini doğrula.
- Register'ları yazıp geri oku.
- MCP2515 loopback mode'da self-test.
- Loopback TX/RX payload eşleşmesini doğrula.
- Gerekli hata durumlarını logla.

Bu fazda OBD, UDS, ISO-TP uygulama mantığı, Nextion UI ve fiziksel araç üzerinde CAN TX implement edilmeyecek.

### Faz 1 — Fiziksel CAN dinleme ve tezgâh TX/RX testi

- Önce fiziksel araç bağlantısında Listen-Only.
- Gelen frame'leri logla; hiçbir frame gönderme.
- Fiziksel bus'ta trafik olmamasını tek başına hata sayma.
- TX/RX testlerini öncelikle loopback veya ayrı bir tezgâh CAN düğümüyle yap.
- Gerçek araçta normal moda geçildiğinde yalnızca sonraki fazlarda tanımlanan diagnostik istekler gönderilsin.
- Rastgele ham CAN TX yapılmasın.

### Faz 2 — OBD-II

- ISO-TP Single Frame temel taşıma.
- Supported PIDs discovery/cache.
- RPM PID `0x0C`.
- RPM decoder.
- Unity unit testleri.
- İlk gerçek diagnostik isteğini tanımlanmış, read-only bir OBD request olarak gönder.

### Faz 3 — Diğer standart PID'ler

Desteklenen PID'lere bağlı olarak:

- hız
- motor yükü
- TPS
- MAF
- IAT
- MAP
- coolant
- ignition advance
- STFT/LTFT
- O2
- fuel/fuel rail pressure
- akü voltajı
- yakıt seviyesi
- katalizör sıcaklığı
- dış sıcaklık
- gaz pedalı D/E
- yağ sıcaklığı
- torque-related standart PID'ler

Doğru formülü doğrulanmamış parametreyi implement edilmiş gibi gösterme.

### Faz 4 — UDS

- Fiziksel adresleme
- `0x10 0x03`
- Pozitif/negatif cevap yönetimi
- `0x3E 0x00`
- `0x22`
- NRC ve `0x78` ResponsePending
- timeout ve session state machine

### Faz 5 — VAG OEM-specific DID profili

- Tek bir doğrulanmış ECU ailesi.
- DID'ler gerçek dokümantasyon veya test cevabıyla doğrulanacak.
- DID/profile decoder sistemi.
- Bilinmeyen DID yok; doğrulanmamışlar `UNKNOWN/TODO`.

### Faz 6 — ONX2432G028 UI genişletme

- Dahili ST7789 panelini başlatan ve temel RPM ekranını çizen bileşen uygulanmıştır.
- RPM yalnızca doğrulanmış OBD cevabından güncellenir; veri yoksa/stale ise `----` gösterilir.
- Daha sonra parametre değerlerinin Parameter Manager üzerinden beslenmesi, dokunmatik giriş ve ek sayfalar eklenecek.

## 12. Kodlama kuralları

- Magic number kullanma; ID, PID, DID, timing ve pinleri merkezi konfigürasyona veya profile taşı.
- C ve C++ dosya uzantılarını gerçek dile uygun seç.
- C'den çağrılacak public header'larda gerekli `extern "C"` korumasını kullan.
- Kütüphane API'sini kaynak koddan doğrula.
- CAN driver arayüzü MCP2515'ten bağımsız kalsın.
- Decoder'ları saf, test edilebilir fonksiyonlar olarak yaz.
- Hata yollarını açık şekilde yönet.
- Kodun derlenmesini doğrula; uyarıları mümkün olduğunca temiz tut.
- Bir donanım testi yapılmadıysa başarılı olduğunu iddia etme.
- Her fazdan sonra değiştirilen dosyaları, yapılan işlemleri, derleme/test sonucunu ve çözülemeyen sorunları bildir.
- Onayım olmadan sonraki faza geçme.

## 13. İlk görev — yalnızca Faz 0

Şu anda yalnızca Faz 0'ın dosya planını hazırla.

İlk yanıtında:

1. Proje kök dizinini ve oluşturulacak dosyaların tam yollarını listele.
2. Her dosyanın neden gerektiğini ve sorumluluğunu bir cümleyle açıkla.
3. MCP2515 repository'sinden alınacak dosyaları ve eklenecek CMake yapılarını açıkça belirt.
4. Faz 0 için gerekli olacak SDK config dosyalarını listele.
5. Kaynak koddan doğrulanması gereken noktaları ayrıca belirt.

**Henüz hiçbir dosyanın içeriğini yazma. Kod bloğu üretme. Faz 0 dışına çıkma. Ben dosya listesini onayladıktan sonra her dosyanın tam ve eksiksiz içeriğini yoluyla birlikte ver.**

## Faz 1 — Gerçek CAN alımı ve OBD-II motor devri sorgusu

- MCP2515 SPI register doğrulaması ve dahili loopback testi Kconfig seçeneği ile açılışta çalıştırılabilir; loopback frame'i fiziksel CAN hattına gönderilmez.
- Varsayılan çalışma modu Listen-Only'dir ve varsayılan derlemede hiçbir CAN frame'i gönderilmez (`CONFIG_OBD_ACTIVE_TX=n`).
- İlk 24 alınan ham frame INFO, sonraki frame'ler DEBUG seviyesinde loglanır. RTR frame'leri ayırt edilir; OBD çözümleyici yalnızca beklenen standart ID ve ISO-TP Single Frame yapısını işler.
- CANSTAT, EFLG, TEC, REC, TXREQ ve RX overflow bilgileri periyodik loglanır; sürekli RX hataları saniyede en fazla bir kez loglanır.
- Aktif sorgu yalnızca kullanıcı `CONFIG_OBD_ACTIVE_TX=y` seçeneğini bilinçli olarak etkinleştirip yeniden derlediğinde açılır.
- Aktif sorgu açıkken Normal moda geçilir; MCP2515 one-shot TX kullanır ve her saniye standart 11-bit CAN ID `0x7DF` ile Mode 01 / PID `0x0C` (motor devri) isteği gönderir.
- `0x7E8`–`0x7EF` cevap ID'leri dinlenir; doğrulanmış ISO-TP Single Frame `41 0C A B` cevabında RPM = `(A * 256 + B) / 4` olarak çözülür.
- Üç ardışık gönderim hatası veya yanıtsızlıktan sonra aktif sorgu durur; alım ve tanılama çalışmaya devam eder.

Bu faz, aracın gerçekten ISO 15765-4, 11-bit ID ve 500 kbit/s kullandığını kanıtlamaz. İlk fiziksel denemede kontak açık ve uygun OBD-CAN bağlantısı gerekir; uygun olmayan araç/protokolde RPM cevabı gelmeyebilir.

### Faz 1b — Pasif CAN gözlemi ve aktif TX filtre kilidi

- Pasif dinlemede toplam alınan frame sayısı, beş saniyelik pencereye göre ortalama frame/s, en fazla 32 benzersiz (ID + STD/EXT) kimlik ve son frame bilgisi periyodik loglanır.
- MCP2515 RX0OVR/RX1OVR sticky bayrakları en fazla 50 ms aralıkla kontrol edilir; görülen bayraklar loglanır, checked SPI işlemiyle temizlenir ve ayrı kümülatif gözlem sayaçları tutulur. Sayaçlar kesin kayıp frame sayısı değildir.
- Listen-Only modunda TEC/REC bitrate doğrulaması olarak kullanılmaz. Geçerli gerçek frame alımı daha güçlü kanıttır; hiç frame gelmemesi tek başına hata değildir.
- `CONFIG_OBD_ACTIVE_TX=y` için standart OBD yanıt maskeleri `0x7F8`, RXF0..RXF5 filtreleri standart `0x7E8` olarak yapılandırılır ve RX buffer filtre modu readback ile kontrol edilir. Filtreler kurulmadan `CAN_MODE_NORMAL` reddedilir.
- ONX2432G028 kartının dahili 320×240 ST7789 LCD ekranında temel RPM UI bulunur. Panel SPI3_HOST üzerinde, MCP2515 ise SPI2_HOST üzerinde çalışır; LCD reseti kart üzerindeki PCF8574 genişleticisiyle, arka aydınlatma GPIO6 ile kontrol edilir. Başlangıçta RPM `----` görünür; yalnızca doğrulanmış OBD RPM yanıtı geldiğinde sayısal değer gösterilir.
