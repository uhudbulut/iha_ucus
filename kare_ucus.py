import csv
import time
import os
from pymavlink import mavutil

print("CSV Dosyasinin Kaydedilecegi Yer:", os.getcwd())

# --- SABITLER VE AYARLAR ---
BAGLANTI = "tcp:127.0.0.1:5762"  # Mission Planner ile cakismayan guvenli API portu
IRTIFA = 10.0
KENAR = 20.0

def mod_ayarla(iha, mod):
    iha.set_mode(mod)
    while iha.flightmode != mod:
        iha.recv_match(type="HEARTBEAT", blocking=True)

def konuma_git(iha, kuzey, dogu, asagi):
    """Kalkis noktasina gore yerel NED koordinatina git (asagi negatif = yukari)"""
    iha.mav.set_position_target_local_ned_send(
        0, iha.target_system, iha.target_component,
        mavutil.mavlink.MAV_FRAME_LOCAL_NED,
        0b110111111000, # yalnizca konum alanlarini kullan
        kuzey, dogu, asagi, 0, 0, 0, 0, 0, 0, 0, 0)

def kayit_al(iha, yazici, baslangic):
    """LOCAL_POSITION_NED mesajini okuyup CSV'ye bir satir yazar, irtifayi dondurur"""
    msg = iha.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
    if msg is None:
        return 0.0  # Kodun None Type hatasi verip cokmesini engeller
    
    # Z ekseni (asagi yon) negatif oldugu icin -1 ile carpiyoruz
    irtifa = msg.z * -1.0 
    
    yazici.writerow([round(time.time() - baslangic, 2),
                     round(msg.x, 2), round(msg.y, 2), round(irtifa, 2)])
                     
    return irtifa

def main():
    # 1. Simulasyon ile baglanti kur
    print(f"{BAGLANTI} uzerinden simulasyona baglanti bekleniyor...")
    iha = mavutil.mavlink_connection(BAGLANTI)
    iha.wait_heartbeat()
    iha.target_system = 1  # KRITIK COZUM: Dronu Sistem 1 olarak zorlar

    iha.mav.request_data_stream_send(iha.target_system, iha.target_component, mavutil.mavlink.MAV_DATA_STREAM_ALL, 2, 1)
    
    print("Baglandi: sistem", iha.target_system)

    # 2. CSV veri dosyasini olustur (with kuralini kaldirdik, dosya cokmeyecek)
    f = open("ucus_kaydi.csv", "w", newline="")
    yazici = csv.writer(f)
    yazici.writerow(["zaman_s", "kuzey_x", "dogu_y", "irtifa_m"])
    baslangic = time.time()

    # 3. Ucus oncesi hazirlik ve Kalkis
    mod_ayarla(iha, "GUIDED")
    iha.arducopter_arm()
    iha.motors_armed_wait()
    print("Motorlar acik, kalkis basliyor...")

    iha.mav.command_long_send(
        iha.target_system, iha.target_component,
        mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, IRTIFA)

    # TODO 1: Kalkis Irtifasini Bekleme
    guncel_irtifa = 0.0
    while guncel_irtifa < (IRTIFA * 0.95):
        okunan_irtifa = kayit_al(iha, yazici, baslangic)
        if okunan_irtifa > 0:
            guncel_irtifa = okunan_irtifa
            
    print(f"Hedef irtifaya ulasildi! Anlik Irtifa: {round(guncel_irtifa, 2)} metre")

    # TODO 2: Karenin 4 kosesini tanimla
    koseler = [
        (KENAR, 0),          # 1. Kose: 20m Kuzey, 0m Dogu
        (KENAR, KENAR),      # 2. Kose: 20m Kuzey, 20m Dogu
        (0, KENAR),          # 3. Kose: 0m Kuzey, 20m Dogu
        (0, 0)               # 4. Kose: Kalkis noktasina donus
    ]

    print("Kare rota basliyor...")

    # TODO 3: Koselere git ve mesafe kontrolu yap
    for kuzey, dogu in koseler:
        konuma_git(iha, kuzey, dogu, -IRTIFA)
        print(f"Hedefe gidiliyor -> Kuzey: {kuzey}, Dogu: {dogu}")
        
        while True:
            # Ucus boyunca telemetri kaydetmeye devam et
            kayit_al(iha, yazici, baslangic)
            
            # Mesafe hesaplamak icin konum verisini cek
            msg = iha.recv_match(type="LOCAL_POSITION_NED", blocking=True, timeout=1)
            if msg is not None:
                mesafe = ((msg.x - kuzey)**2 + (msg.y - dogu)**2)**0.5
                
                # Hedefe 1 metreden fazla yaklasildiysa siradaki koseye gec
                if mesafe < 1.0:
                    print(f"Koseye ulasildi! Koordinat: ({kuzey}, {dogu})")
                    break

    # 4. Rota bittiginde inis moduna gec
    print("Tum koseler tamamlandi, inis basliyor...")
    mod_ayarla(iha, "LAND")
    
    # Yere inene kadar kayit almaya devam et
    while guncel_irtifa > 0.2:
        okunan_irtifa = kayit_al(iha, yazici, baslangic)
        if okunan_irtifa > 0:
            guncel_irtifa = okunan_irtifa
            
    print("Inis basariyla tamamlandi. Ucus bitti!")
    f.close()  # CSV dosyasini guvenle kayit edip kapatiyoruz

if __name__ == "__main__":
    main()
