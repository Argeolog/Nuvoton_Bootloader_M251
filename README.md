# Nuvoton_Bootloader_M251
Nuvoton Bootloader (For LDROM)
Bu program aracılıgı ile yazılım atabilirsiniz.
https://github.com/Argeolog/Bootloader_Hex_Send

Not: Güvenlik protokolü vardır.
Cihaza Hangi hex dosyasını kabul edeceğini belirtebilirsiniz. Cihaz adı yaz butonu ile cihaza bu ismi yazdırın. Ardından İşlemci içindeki yazılım ile yazdırmak istediğiniz yazılım aynı mı kontrol ettirebilirsiniz.
Böylece X cihazına Y yazılımın atılmasına engel olursunuz.

https://youtu.be/TSw3nBcLB3A


Not:
Scatter File Hatasında;

Keil>Project>options for target>linker
Menüsünde Scatter File dosyasını seçin.
Bu Dosya : \Library\Device\Nuvoton\M251\Source\ARM Klasoründe Yer alır.

Projeyi ilk derlediğinizde Library Hatası alacaksınız. Bunun sebebi Ben Libraryleri
D:\Software Tools\Keil\Nuvoton\Library Yolunda tuttuğum için.
Ya bu yolu oluştutup library klasörünüzü buraya kopyalayın veya
Keil>Project>options for target>C/C++(AC6) Sekmesinde include Paths yollarını düzeltin.

Ürünlerin Vendor ve Product ID leri 0x7E6

Flash Aynı Zamanda Eeprom olarak kullanılmaktadır.
3 KB Eprom A ayrılmıştır.
29 KB Alan program için ayrılmıştır.

Kodunuz 29 KB geçer ise, Eprom için ayrılan yere yazacaktır.

Değişkenler.h dosyasında açıklama bulacaksınız.

// 32 KB Flashi olan islemci toplamda 64 sayfa(page) dan olusmaktadir.
// Biz 29 KB ini program için kullaniyoruz Buda 57 Sayfa yapar.
// 58. Sayfa Baslangici Cihazin ayarlarinin tutuldugu alandir. Bu adresin Baslangiç Adresi 0x7200 dür.
// Ayni Zamanda 0x7200 adresinin ilk byte'i Cihazin boot moduna girip girmeyecegini belirler. Burasi 0 degil ise cihaz boot modunda açilacaktir.
// Flash Yazilmaya baslamadan önce ilk 57 Sayfayi siler ve Eeprom bölümüne dokunmaz.

