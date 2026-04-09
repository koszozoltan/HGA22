HGA22 Time Signal Receiver
This project is used to receive and decode the time signals broadcasted at 135.6 kHz by the HGA22 transmitter station in Lakihegy. The system is capable of processing phase-shift keying (FSK) signals and extracting time information.

https://github.com/5an2i/USB_HGA22

📋 Features
Frequency: 135.6 kHz (LF band)

Modulation: FSK

Platform: STM32C071 Nucleo
Data format: Serial, RNDIS USB - NTP
🛠️ Hardware architecture
The following components were required for reception:

Antenna: [Ferrite rod or RFID reader antenna] tuned to 135.6 kHz.



HGA22 Time Signal Receiver
Ez a projekt a lakihegyi HGA22 adóállomás 135,6 kHz-en sugárzott pontos idő jeleinek vételére és dekódolására szolgál. A rendszer képes a fázismodulált (FSK) jelek feldolgozására és az időinformáció kinyerésére.

📋 Jellemzők
Frekvencia: 135,6 kHz (LF sáv)

Moduláció: FSK

Platform: STM32C071 Nucleo
Adatformátum: Soros , RNDIS USB - NTP 
🛠️ Hardver felépítés
A vételhez az alábbi komponensekre volt szükség:

Antenna: [Ferrit rúd vagy RFID olvasó antenna] 135,6 kHz-re hangolva.

1. Mi az a HGA22?
A HGA22 egy hosszúhullámú (LF) rádióadó Lakihegyen, amely 135,6 kHz-en sugároz. Ez az EFR (Europäische Funk-Rundsteuerung) hálózat része, amit elsősorban távvezérlésre (pl. utcai világítás, „éjszakai áram”) és pontos idő szinkronizációra használnak.

Frekvencia: 135,6 kHz

Adatsebesség: 200 bps (Baud)

Szerepe: Alternatíva a német DCF77-re, de nagyobb adatsebességgel és gyakrabban küldött időinformációval rendelkezik (akár 10 másodpercenként).

2. A Moduláció: FSK (Frequency Shift Keying)
A digitális adatokat frekvenciabillentyűzéssel kódolják. Ez azt jelenti, hogy a logikai '0' és '1' szinteket két különböző frekvencia képviseli a vivőfrekvencia körül.

Shift (löket): ±170 Hz

Frekvenciák: * F_low: 135,430 kHz (Space / 0)

F_high: 135,770 kHz (Mark / 1)
A vevő dolga, hogy eldöntse: éppen melyik frekvencia „szól”.

3. A Matematikai Motor: Goertzel-algoritmus
Hogyan detektáljunk egy konkrét frekvenciát kevés számítási kapacitással? Itt jön a Goertzel-algoritmus. Ez egy speciális IIR (Infinite Impulse Response) szűrő, amely a Diszkrét Fourier-transzformáció (DFT) egyetlen komponensét számolja ki.

Miért jobb, mint az FFT?
Mivel nekünk csak két konkrét frekvenciát (a löket alatti és feletti értékeket) kell figyelnünk, felesleges a teljes spektrumot kiszámolni. A Goertzel sokkal gyorsabb mikrokontrolleren.

A kódunkban két Goertzel szűrő fut párhuzamosan. Amelyik kimenetén nagyobb az „energia” (amplitúdó), azt a bitet tekintjük érvényesnek.

4. HGA22 Frame Parsolás (Keretezés)
Az adatok nem ömlesztve jönnek, hanem kötött szerkezetű táviratokban (frames). A protokoll (hasonlóan a Versacom-hoz) az alábbi módon épül fel:

Start karakter: 0x68 (ez jelzi a csomag elejét).

Hossz: Megadja, hány bájt következik.

Ismételt Hossz és Start: Biztonsági ellenőrzés.

Címzés: Megadja, kinek szól az üzenet (az időjel címe általában 0x00).

Adat (Payload): Itt van a lényeg! BCD kódolással tartalmazza az évet, hónapot, napot, órát, percet és másodpercet.

Checksum (CRC): Hibaellenőrzés.

Stop karakter: 0x16.

A kódban (main.cpp):
A parsoló egy állapotgépet (State Machine) használ. Addig dobálja el a bejövő bájtokat, amíg nem talál egy 0x68-at, majd elkezdi feltölteni a puffert a megadott hossz alapján.


https://cmake.org/download/

https://www.codeblocks.org/

https://developer.arm.com/downloads/-/arm-gnu-toolchain-downloads

https://sourceforge.net/projects/gnuwin32/


