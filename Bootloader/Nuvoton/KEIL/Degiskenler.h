#define FlashYaz 150
#define FlashSil 151
#define CihazAyarFlashSil 152
#define YazmaTamamlandi 153
#define CihazAdiEslestir 154
#define CihazAdiYaz 155
#define CihazAdiOku 156


#define BootModeAdres 0x7200

#define ProgramSayfaSayisi 57 // Program 57 Sayfadan Olusuyor.Dolayisi ile Veri Yazmadan �nce 57 Sayfayi Temizliyoruz.
#define CihazAyarPageStart 0x7200
#define CihazAyarPageEnd   0x7400
#define CihazAdiAdres 0x7204


// 32 KB Flashi olan islemci toplamda 64 sayfa(page) dan olusmaktadir.
// Biz 29 KB ini program i�in kullaniyoruz Buda 57 Sayfa yapar.
// 58. Sayfa Baslangici Cihazin ayarlarinin tutuldugu alandir. Bu adresin Baslangi� Adresi 0x7200 d�r.
// Ayni Zamanda 0x7200 adresinin ilk byte'i Cihazin boot moduna girip girmeyecegini belirler. Burasi 0 degil ise cihaz boot modunda a�ilacaktir.
// Flash Yazilmaya baslamadan �nce ilk 57 Sayfayi siler ve Eeprom b�l�m�ne dokunmaz.